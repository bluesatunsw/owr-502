#pragma once

#include <linux/can.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <type_traits>
#include <unordered_map>

namespace bg431esc1_actuator {
struct [[gnu::packed]] CanId {
  enum class Priority : std::uint8_t {
    kExceptional = 0,
    kImmediate = 1,
    kFast = 2,
    kHigh = 3,
    kNominal = 4,
    kLow = 5,
    kSlow = 6,
    kOptional = 7,
  };

  using DeviceClass = std::uint8_t;
  using DeviceIndex = std::uint8_t;
  using ApiIndex = std::uint16_t;

  Priority priority : 3;
  uint8_t anonymous : 1 = 0b0;
  DeviceClass device_class : 6;
  ApiIndex api_index : 12;
  uint8_t reserved : 1 = 0b1;
  DeviceIndex device_index : 6;
};

template <typename T>
std::optional<T> from_bytes(std::span<const std::byte> bytes)
  requires(std::is_trivially_copyable_v<T>)
{
  if (bytes.size() != sizeof(T)) return std::nullopt;

  // Use this instead of a reinterpret cast deref to ensure proper alignment
  T res;
  std::copy_n(bytes.begin(), sizeof(T), reinterpret_cast<std::byte*>(&res));
  return res;
}

class CanMux {
 private:
  struct ConnectionIndex {
    bool operator==(const ConnectionIndex&) const = default;

    CanId::DeviceClass device_class;
    CanId::DeviceIndex device_index;
  };

  struct ConnectionIndexHasher {
    std::size_t operator()(const ConnectionIndex& index) const noexcept {
      return std::hash<int>{}((0xFF * index.device_class) + index.device_index);
    }
  };

 public:
  class Connection {
    friend CanMux;

   public:
    using RecvCallback =
        std::function<void(CanId::ApiIndex, std::span<const std::byte>)>;

    Connection() = default;

    template <typename T>
    void send(CanId::ApiIndex api_index, const T& data,
              CanId::Priority priority)
      requires(std::is_trivially_copyable_v<T> && sizeof(T) <= CANFD_MAX_DLEN)
    {
      raw_send(api_index,
               std::bit_cast<const std::array<std::byte, sizeof(T)>>(data),
               priority);
    }

    void bind(RecvCallback&& callback);

    std::mutex& mutex();

    Connection(const CanMux&) = delete;
    Connection& operator=(const CanMux&) = delete;

    Connection(Connection&& other);
    Connection& operator=(Connection&& other);

    ~Connection();

   private:
    void raw_send(CanId::ApiIndex api_index, std::span<const std::byte> data,
                  CanId::Priority priority);

    Connection(CanMux& mux, ConnectionIndex index);

    std::mutex m_mutex;
    RecvCallback m_recv_cb;
    CanMux* m_mux{};
    ConnectionIndex m_index{};
  };

  static CanMux& get_instance();

  Connection open(CanId::DeviceClass device_class,
                  CanId::DeviceIndex device_index);

  CanMux(const CanMux&) = delete;
  CanMux& operator=(const CanMux&) = delete;

  CanMux(CanMux&&) = delete;
  CanMux& operator=(CanMux&&) = delete;

 private:
  CanMux();

  void recieve();

  std::mutex m_mutex;
  std::unordered_map<ConnectionIndex, Connection*, ConnectionIndexHasher>
      m_connections;
  std::jthread m_worker;
  int m_fd{};
};
}  // namespace bg431esc1_actuator
