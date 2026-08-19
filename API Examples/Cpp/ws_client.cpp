/*
 * ws_client.cpp - Minimal WebSocket client implementation
 */

#include "ws_client.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
namespace {
    struct WsaInit {
        WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2,2), &d); }
        ~WsaInit() { WSACleanup(); }
    };
    static WsaInit g_wsaInit;
}
#else
#  include <fcntl.h>
#  include <sys/time.h>
#endif

/* -------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/
static uint32_t randomU32() {
    static std::mt19937 rng(static_cast<uint32_t>(std::time(nullptr)));
    std::uniform_int_distribution<uint32_t> dist;
    return dist(rng);
}

std::string WebSocketClient::base64Encode(const uint8_t* data, size_t len) {
    static const char* tbl =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t n = (uint32_t)data[i] << 16;
        if (i + 1 < len) n |= (uint32_t)data[i + 1] << 8;
        if (i + 2 < len) n |= (uint32_t)data[i + 2];
        out += tbl[(n >> 18) & 63];
        out += tbl[(n >> 12) & 63];
        out += (i + 1 < len) ? tbl[(n >> 6) & 63] : '=';
        out += (i + 2 < len) ? tbl[n & 63]        : '=';
    }
    return out;
}

std::string WebSocketClient::generateWebSocketKey() {
    uint8_t bytes[16];
    for (int i = 0; i < 4; ++i) {
        uint32_t r = randomU32();
        std::memcpy(bytes + i * 4, &r, 4);
    }
    return base64Encode(bytes, 16);
}

/* -------------------------------------------------------------------------
 * Constructor / destructor
 * -------------------------------------------------------------------------*/
WebSocketClient::WebSocketClient() = default;

WebSocketClient::~WebSocketClient() { disconnect(); }

void WebSocketClient::onMessage(MessageCb cb) { m_onMessage = std::move(cb); }
void WebSocketClient::onError(ErrorCb cb)     { m_onError   = std::move(cb); }

/* -------------------------------------------------------------------------
 * connect()
 * -------------------------------------------------------------------------*/
bool WebSocketClient::connect(const std::string& host, int port,
                              const std::string& path, int /*timeoutMs*/)
{
    if (m_connected) return true;

    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    std::string portStr = std::to_string(port);
    if (getaddrinfo(host.c_str(), portStr.c_str(), &hints, &res) != 0 || !res) {
        if (m_onError) m_onError("DNS lookup failed for " + host);
        return false;
    }

    m_sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (m_sock == INVALID_SOCK) {
        freeaddrinfo(res);
        if (m_onError) m_onError("Failed to create socket");
        return false;
    }

    if (::connect(m_sock, res->ai_addr, (int)res->ai_addrlen) != 0) {
        freeaddrinfo(res);
        CLOSE_SOCK(m_sock);
        m_sock = INVALID_SOCK;
        if (m_onError) m_onError("TCP connect failed to " + host);
        return false;
    }
    freeaddrinfo(res);

    if (!performHandshake(host, port, path)) {
        CLOSE_SOCK(m_sock);
        m_sock = INVALID_SOCK;
        if (m_onError) m_onError("WebSocket handshake failed");
        return false;
    }

    m_connected = true;
    m_running   = true;
    m_recvThread = std::thread(&WebSocketClient::receiveLoop, this);
    return true;
}

/* -------------------------------------------------------------------------
 * HTTP upgrade handshake
 * -------------------------------------------------------------------------*/
bool WebSocketClient::performHandshake(const std::string& host, int port,
                                       const std::string& path)
{
    std::string key = generateWebSocketKey();

    std::ostringstream req;
    req << "GET " << path << " HTTP/1.1\r\n"
        << "Host: " << host << ":" << port << "\r\n"
        << "Upgrade: websocket\r\n"
        << "Connection: Upgrade\r\n"
        << "Sec-WebSocket-Key: " << key << "\r\n"
        << "Sec-WebSocket-Version: 13\r\n"
        << "\r\n";
    std::string reqStr = req.str();
    if (!sendAll(reqStr.data(), reqStr.size())) return false;

    /* Read HTTP response headers line by line */
    std::string response;
    char c;
    while (response.size() < 8192) {
        if (!recvAll(&c, 1)) return false;
        response += c;
        if (response.size() >= 4 &&
            response.substr(response.size() - 4) == "\r\n\r\n") break;
    }

    return response.find("101") != std::string::npos &&
           response.find("Switching Protocols") != std::string::npos;
}

/* -------------------------------------------------------------------------
 * disconnect()
 * -------------------------------------------------------------------------*/
void WebSocketClient::disconnect() {
    m_running   = false;
    m_connected = false;

    if (m_sock != INVALID_SOCK) {
        /* Send a close frame if we can, then shut the socket */
        sendFrame("", 0x08);
#ifdef _WIN32
        shutdown(m_sock, SD_BOTH);
#else
        ::shutdown(m_sock, SHUT_RDWR);
#endif
        CLOSE_SOCK(m_sock);
        m_sock = INVALID_SOCK;
    }

    if (m_recvThread.joinable()) m_recvThread.join();
}

/* -------------------------------------------------------------------------
 * send() — public thread-safe method
 * -------------------------------------------------------------------------*/
bool WebSocketClient::send(const std::string& text) {
    return sendFrame(text, 0x01);
}

/* -------------------------------------------------------------------------
 * sendFrame() — builds a masked WebSocket frame (clients MUST mask)
 * -------------------------------------------------------------------------*/
bool WebSocketClient::sendFrame(const std::string& payload, uint8_t opcode) {
    std::lock_guard<std::mutex> lock(m_sendMtx);
    if (m_sock == INVALID_SOCK) return false;

    size_t len = payload.size();
    std::vector<uint8_t> frame;
    frame.reserve(10 + len);

    frame.push_back(0x80 | (opcode & 0x0F));           /* FIN + opcode */

    if (len < 126) {
        frame.push_back(0x80 | (uint8_t)len);          /* MASK + length */
    } else if (len < 65536) {
        frame.push_back(0x80 | 126);
        frame.push_back((uint8_t)(len >> 8));
        frame.push_back((uint8_t)(len & 0xFF));
    } else {
        frame.push_back(0x80 | 127);
        for (int i = 7; i >= 0; --i)
            frame.push_back((uint8_t)((len >> (i * 8)) & 0xFF));
    }

    /* 4-byte masking key */
    uint32_t maskKey = randomU32();
    uint8_t mask[4];
    std::memcpy(mask, &maskKey, 4);
    frame.insert(frame.end(), mask, mask + 4);

    /* Masked payload */
    for (size_t i = 0; i < len; ++i)
        frame.push_back((uint8_t)payload[i] ^ mask[i & 3]);

    return sendAll(frame.data(), frame.size());
}

/* -------------------------------------------------------------------------
 * readFrame() — reads one WebSocket frame from the socket
 * -------------------------------------------------------------------------*/
bool WebSocketClient::readFrame(std::string& payloadOut, uint8_t& opcodeOut, bool& finOut) {
    uint8_t header[2];
    if (!recvAll(header, 2)) return false;

    finOut   = (header[0] & 0x80) != 0;
    opcodeOut = header[0] & 0x0F;

    bool masked    = (header[1] & 0x80) != 0;
    uint64_t plen  = header[1] & 0x7F;

    if (plen == 126) {
        uint8_t ext[2];
        if (!recvAll(ext, 2)) return false;
        plen = ((uint64_t)ext[0] << 8) | ext[1];
    } else if (plen == 127) {
        uint8_t ext[8];
        if (!recvAll(ext, 8)) return false;
        plen = 0;
        for (int i = 0; i < 8; ++i) plen = (plen << 8) | ext[i];
    }

    uint8_t maskKey[4] = {};
    if (masked) {
        if (!recvAll(maskKey, 4)) return false;
    }

    std::vector<uint8_t> buf(static_cast<size_t>(plen));
    if (plen > 0 && !recvAll(buf.data(), buf.size())) return false;

    if (masked) {
        for (size_t i = 0; i < buf.size(); ++i)
            buf[i] ^= maskKey[i & 3];
    }

    payloadOut.assign(reinterpret_cast<char*>(buf.data()), buf.size());
    return true;
}

/* -------------------------------------------------------------------------
 * receiveLoop() — background thread
 * -------------------------------------------------------------------------*/
void WebSocketClient::receiveLoop() {
    std::string accumulated;

    while (m_running) {
        std::string payload;
        uint8_t opcode = 0;
        bool fin       = false;

        if (!readFrame(payload, opcode, fin)) {
            if (m_running.exchange(false)) {
                m_connected = false;
                if (m_onError) m_onError("Connection lost");
            }
            break;
        }

        switch (opcode) {
            case 0x00: /* continuation */
                accumulated += payload;
                if (fin && m_onMessage) {
                    m_onMessage(accumulated);
                    accumulated.clear();
                }
                break;

            case 0x01: /* text */
            case 0x02: /* binary */
                if (fin) {
                    if (m_onMessage) m_onMessage(payload);
                } else {
                    accumulated = payload;
                }
                break;

            case 0x08: /* close */
                m_running   = false;
                m_connected = false;
                break;

            case 0x09: /* ping — respond with pong */
                sendFrame(payload, 0x0A);
                break;

            default:
                break;
        }
    }
}

/* -------------------------------------------------------------------------
 * recvAll / sendAll — reliable read/write wrappers
 * -------------------------------------------------------------------------*/
bool WebSocketClient::recvAll(void* buf, size_t len) {
    char* ptr = static_cast<char*>(buf);
    size_t got = 0;
    while (got < len) {
        int n = recv(m_sock, ptr + got, (int)(len - got), 0);
        if (n <= 0) return false;
        got += (size_t)n;
    }
    return true;
}

bool WebSocketClient::sendAll(const void* buf, size_t len) {
    const char* ptr = static_cast<const char*>(buf);
    size_t sent = 0;
    while (sent < len) {
        int n = ::send(m_sock, ptr + sent, (int)(len - sent), 0);
        if (n <= 0) return false;
        sent += (size_t)n;
    }
    return true;
}
