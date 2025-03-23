#include "bg431esc1_actuator/can_mux.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cstring>
#include <format>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>

namespace bg431esc1_actuator {
void CanMux::Connection::bind(RecvCallback&& callback) {
  std::scoped_lock guard{m_mutex};
  if (!m_mux) throw std::logic_error("Use of moved from/null connection.");
  m_recv_cb = std::move(callback);
}

std::mutex& CanMux::Connection::mutex() { return m_mutex; }

CanMux::Connection::Connection(Connection&& other) {
  if (!other.m_mux) return;
  std::scoped_lock guard{other.m_mux->m_mutex, other.m_mutex};

  m_recv_cb = std::move(other.m_recv_cb);
  m_mux = other.m_mux;
  m_index = other.m_index;

  if (other.m_mux) {
    other.m_mux = nullptr;
    m_mux->m_connections.at(m_index) = this;
  }
}

CanMux::Connection& CanMux::Connection::operator=(Connection&& other) {
  if (this == &other) return *this;

  if (m_mux) {
    std::scoped_lock guard{m_mux->m_mutex, m_mutex};
    m_mux->m_connections.erase(m_index);
  }
  if (other.m_mux) {
    std::scoped_lock guard{m_mutex, other.m_mux->m_mutex, other.m_mutex};

    m_recv_cb = std::move(other.m_recv_cb);
    m_mux = other.m_mux;
    m_index = other.m_index;

    other.m_mux = nullptr;
    m_mux->m_connections.at(m_index) = this;
  }

  return *this;
}

CanMux::Connection::~Connection() {
  if (!m_mux) return;
  std::scoped_lock guard{m_mux->m_mutex, m_mutex};
  m_mux->m_connections.erase(m_index);
  m_mux = nullptr;
}

void CanMux::Connection::raw_send(CanId::ApiIndex api_index,
                                  std::span<const std::byte> data,
                                  CanId::Priority priority) {
  if (!m_mux) throw std::logic_error("Use of moved from/null connection.");

  can_frame frame{};
  frame.can_id = std::bit_cast<canid_t>(CanId{
      .device_index = m_index.device_index,
      .api_index = api_index,
      .device_class = m_index.device_class,
      .priority = priority,
  });
  frame.len = static_cast<std::uint8_t>(data.size());
  std::ranges::copy(data, reinterpret_cast<std::byte*>(frame.data));

  if (write(m_mux->m_fd, &frame, sizeof(frame)) == -1)
    throw std::runtime_error(
        std::format("Failed to send frame, error: {}",
                    std::string_view{std::strerror(errno)}));
}

CanMux::Connection::Connection(CanMux& mux, ConnectionIndex index)
    : m_mux{&mux}, m_index{index} {
  std::scoped_lock guard{mux.m_mutex};
  auto [iter, inserted]{mux.m_connections.emplace(
      std::piecewise_construct, std::forward_as_tuple(m_index),
      std::forward_as_tuple(this))};
  if (!inserted) {
    throw std::runtime_error(
        std::format("Duplicate CAN device: class={}, index={}",
                    index.device_class, index.device_index));
  }
}

CanMux& CanMux::get_instance() {
  // Thread safe per the C++ spec
  static CanMux instance;
  return instance;
}

CanMux::Connection CanMux::open(CanId::DeviceClass device_class,
                                CanId::DeviceIndex device_index) {
  // The connection object handles updating m_connections
  return Connection{*this, {device_class, device_index}};
}

CanMux::CanMux() {
  using namespace std::literals;

  m_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_fd == -1)
    throw std::runtime_error(
        std::format("CAN mux failed to create socket, error: {}",
                    std::string_view{std::strerror(errno)}));

  ifreq ifr{};
  std::string_view kInterface{"can0\0"};
  std::copy(kInterface.cbegin(), kInterface.cend(),
            static_cast<char*>(ifr.ifr_name));
  if (ioctl(m_fd, SIOCGIFINDEX, &ifr) == -1)
    throw std::runtime_error(
        std::format("CAN mux failed to find the index of the CAN interface "
                    "device, error: {}",
                    std::string_view{std::strerror(errno)}));

  
  sockaddr_can addr{
      .can_family = AF_CAN,
      .can_ifindex = ifr.ifr_ifindex,
      .can_addr = {},
  };
  if (bind(m_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1)
    throw std::runtime_error(
        std::format("CAN mux failed to bind socket, error: {}",
                    std::string_view{std::strerror(errno)}));

  // Enable timeout
  timeval timeout{.tv_sec = 1, .tv_usec = 0};
  if (0 != setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)))
    throw std::runtime_error(
        std::format("CAN mux failed to set timeout, error: {}",
                    std::string_view{std::strerror(errno)}));
 
  m_worker = std::jthread{[this](std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
      recieve();
    }
  }};

  std::cerr << "CAN mux initialised fd: " << m_fd << std::endl;
}

void CanMux::recieve() {
  canfd_frame frame{};
  if (read(m_fd, &frame, sizeof(frame)) == -1) {
    // On timeout return are check if the worker should be stopped
    if (errno == EAGAIN || errno == EWOULDBLOCK) return;
    throw std::runtime_error(
        std::format("CAN Mux failed to invoke recv, error: {}",
                    std::string_view{std::strerror(errno)}));
  }

  auto id{std::bit_cast<CanId>(frame.can_id)};

  // If neither the mux nor connection lock is held (even for a moment) there is
  // a chance that the Connection may get destroyed, so manually lock/unlock to
  // ensure overlap
  m_mutex.lock();
  auto conn_iter{m_connections.find({id.device_class, id.device_index})};
  if (conn_iter == m_connections.end()) {
    m_mutex.unlock();
    std::cerr
        << std::format(
               "No connection associated with CAN device: class={}, index={}",
               static_cast<CanId::DeviceClass>(id.device_class),
               static_cast<CanId::DeviceIndex>(id.device_index))
        << std::endl;
    return;
  }
  auto conn{conn_iter->second};
  std::scoped_lock guard{conn->m_mutex};
  m_mutex.unlock();

  if (!conn->m_recv_cb) {
    std::cerr << std::format(
                     "No callback bound to CAN device: class={}, index={}",
                     static_cast<CanId::DeviceClass>(id.device_class),
                     static_cast<CanId::DeviceIndex>(id.device_index))
              << std::endl;
    return;
  }

  conn->m_recv_cb(id.api_index,
                  {reinterpret_cast<const std::byte*>(frame.data), frame.len});
}
}  // namespace bg431esc1_actuator
