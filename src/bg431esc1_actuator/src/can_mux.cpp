#include "bg431esc1_actuator/can_mux.hpp"

#include <linux/can.h>
#include <sys/socket.h>

#include <algorithm>
#include <bit>
#include <format>
#include <mutex>
#include <stdexcept>
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
  std::scoped_lock guard{other.m_mux->m_mutex, other.m_mutex};

  m_recv_cb = std::move(other.m_recv_cb);
  m_mux = other.m_mux;
  m_index = other.m_index;

  other.m_mux = nullptr;

  if (!other.m_mux) return;
  m_mux->m_connections.at(m_index) = this;
}

CanMux::Connection& CanMux::Connection::operator=(Connection&& other) {
  if (this == &other) return *this;

  {
    std::scoped_lock guard{m_mux->m_mutex, m_mutex};
    m_mux->m_connections.erase(m_index);
  }
  {
    std::scoped_lock guard{m_mutex, other.m_mux->m_mutex, other.m_mutex};

    m_recv_cb = std::move(other.m_recv_cb);
    m_mux = other.m_mux;
    m_index = other.m_index;

    other.m_mux = nullptr;

    if (!other.m_mux) return *this;
    m_mux->m_connections.at(m_index) = this;
  }

  return *this;
}

CanMux::Connection::~Connection() {
  std::scoped_lock guard{m_mux->m_mutex, m_mutex};
  if (!m_mux) return;
  m_mux->m_connections.erase(m_index);
  m_mux = nullptr;
}

void CanMux::Connection::raw_send(CanId::ApiIndex api_index,
                                  std::span<const std::byte> data,
                                  CanId::Priority priority) {
  if (!m_mux) throw std::logic_error("Use of moved from/null connection.");

  canfd_frame frame{};
  frame.can_id = std::bit_cast<canid_t>(CanId{
      .priority = priority,
      .device_class = m_index.device_class,
      .api_index = api_index,
      .device_index = m_index.device_index,
  });
  frame.len = static_cast<std::uint8_t>(data.size());
  frame.flags = 0;
  std::ranges::copy(data, reinterpret_cast<std::byte*>(frame.data));

  if (::send(m_mux->m_fd, &frame, sizeof(frame), 0) == -1)
    throw std::runtime_error("Failed to send frame");
  ;
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
  m_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (m_fd == -1) throw std::runtime_error("CAN Mux failed to create socket");

  sockaddr_can addr{
      .can_family = AF_CAN,
      .can_ifindex = 0,
      .can_addr = {},
  };
  if (bind(m_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1)
    throw std::runtime_error("CAN Mux failed to bind socket");

  m_worker = std::jthread{[this] {
    while (true) {
      recieve();
    }
  }};
}

void CanMux::recieve() {
  canfd_frame frame{};
  if (recv(m_fd, &frame, sizeof(frame), 0) == -1)
    throw std::runtime_error("CAN Mux failed to invoke recv.");

  auto id{std::bit_cast<CanId>(frame.can_id)};

  // If neither the mux nor connection lock is held (even for a moment) there is
  // a chance that the Connection may get destroyed, so manually lock/unlock to
  // ensure overlap
  m_mutex.lock();
  auto conn_iter{m_connections.find({id.device_class, id.device_index})};
  if (conn_iter == m_connections.end()) {
    m_mutex.unlock();
    // TODO: Warn unhandled
    return;
  }
  auto conn{conn_iter->second};
  std::scoped_lock guard{conn->m_mutex};
  m_mutex.unlock();

  if (!conn->m_recv_cb) {
    // TODO: Warn unbound
    return;
  }

  conn->m_recv_cb(id.api_index,
                  {reinterpret_cast<const std::byte*>(frame.data), frame.len});
}
}  // namespace bg431esc1_actuator
