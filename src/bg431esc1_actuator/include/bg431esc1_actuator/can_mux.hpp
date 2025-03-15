#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <linux/can.h>
#include <mutex>
#include <span>
#include <sys/socket.h>
#include <thread>
#include <type_traits>
#include <unordered_map>


namespace hardware_interface {
struct [[gnu::packed]] CanId {
  enum class Priority : std::uint8_t {

  };

  using ApiIndex = std::uint16_t;

  Priority priority: 3;
  uint8_t anonymous: 1 = 0b0;
  uint8_t device_class: 6;
  ApiIndex api_index: 12;
  uint8_t reserved: 1 = 0b1;
  uint8_t address: 6;
};

class CanMux {
  private:
    struct ConnectionIndex {
        bool operator==(const ConnectionIndex&) const = default;

        std::uint8_t device_class;
        std::uint8_t address;
    };

    struct ConnectionIndexHasher {
        std::size_t operator()(const ConnectionIndex& index) const noexcept {
          return std::hash<int>{}((0xFF * index.device_class) + index.address);
        }
    };

  public:
    class Connection {
        friend CanMux;

      public:
        using RecvCallback = std::function<void(CanId::ApiIndex, std::span<const std::byte>)>;

        Connection() = default;

        template<typename T>
        void send(CanId::ApiIndex api_index, const T& data, CanId::Priority priority) requires (std::is_trivially_copyable_v<T> && sizeof(T) <= CANFD_MAX_DLEN)  {
          raw_send(api_index, std::bit_cast<const std::array<std::byte, sizeof(T)>>(data), priority);
        }

        void bind(RecvCallback&& callback);

        Connection(const CanMux&) = delete;
        Connection& operator=(const CanMux&) = delete;
    
        Connection(Connection&& other);
        Connection& operator=(Connection&& other);

        ~Connection();

      private:
        void raw_send(CanId::ApiIndex api_index, std::span<const std::byte> data, CanId::Priority priority);

        Connection(CanMux& mux, ConnectionIndex index);

        std::mutex m_mutex;
        RecvCallback m_recv_cb;
        CanMux* m_mux{};
        ConnectionIndex m_index{};
    };

    static CanMux& get_instance();

    Connection open(std::uint8_t device_class, std::uint8_t address);

    CanMux(const CanMux&) = delete;
    CanMux& operator=(const CanMux&) = delete;

    CanMux(CanMux&&) = delete;
    CanMux& operator=(CanMux&&) = delete;

  private:
    CanMux();

    void recieve();

    std::mutex m_mutex;
    std::unordered_map<ConnectionIndex, Connection*, ConnectionIndexHasher> m_connections;
    std::jthread m_worker;
    int m_fd{};
};
}

