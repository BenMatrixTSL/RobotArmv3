/*
 * ws_client.h - Minimal WebSocket client (RFC 6455, text frames only)
 *
 * No external dependencies beyond the standard library and platform sockets.
 * On Windows: links against ws2_32.lib (handled automatically via #pragma).
 * On POSIX   : link with no extra flags.
 */
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
   using SocketHandle = SOCKET;
#  define INVALID_SOCK INVALID_SOCKET
#  define CLOSE_SOCK(s) closesocket(s)
#else
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <sys/socket.h>
#  include <unistd.h>
   using SocketHandle = int;
#  define INVALID_SOCK (-1)
#  define CLOSE_SOCK(s) ::close(s)
#endif

/* -------------------------------------------------------------------------
 * WebSocketClient
 *
 * Thread-safe for concurrent send() calls.
 * Fires onMessage() from a background receive thread; do not call disconnect()
 * from inside that callback.
 * -------------------------------------------------------------------------*/
class WebSocketClient {
public:
    using MessageCb = std::function<void(const std::string&)>;
    using ErrorCb   = std::function<void(const std::string&)>;

    WebSocketClient();
    ~WebSocketClient();

    // Register callbacks before calling connect().
    void onMessage(MessageCb cb);
    void onError(ErrorCb cb);

    // Connect and perform the WebSocket upgrade handshake.
    // Returns true on success. timeoutMs applies to the TCP connect only.
    bool connect(const std::string& host, int port,
                 const std::string& path = "/", int timeoutMs = 10000);

    // Send a UTF-8 text frame. Thread-safe.
    bool send(const std::string& text);

    // Disconnect and stop the receive thread.
    void disconnect();

    bool isConnected() const { return m_connected.load(); }

private:
    /* Receive loop runs on m_recvThread */
    void receiveLoop();

    /* Low-level frame I/O */
    bool sendFrame(const std::string& payload, uint8_t opcode = 0x01);
    bool readFrame(std::string& payloadOut, uint8_t& opcodeOut, bool& finOut);

    /* Reliable read/write wrappers */
    bool recvAll(void* buf, size_t len);
    bool sendAll(const void* buf, size_t len);

    /* HTTP upgrade helpers */
    bool performHandshake(const std::string& host, int port, const std::string& path);
    static std::string base64Encode(const uint8_t* data, size_t len);
    static std::string generateWebSocketKey();

    SocketHandle       m_sock     { INVALID_SOCK };
    std::atomic<bool>  m_connected{ false };
    std::atomic<bool>  m_running  { false };
    std::thread        m_recvThread;
    std::mutex         m_sendMtx;
    MessageCb          m_onMessage;
    ErrorCb            m_onError;
};
