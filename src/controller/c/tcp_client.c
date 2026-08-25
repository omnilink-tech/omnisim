/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
 */

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <omnisim/types.h>

#endif

#include "tcp_client.h"

const int ERROR_BUFFER_SIZE = 256;

int tcp_client_new(const char *host, int port, char *buffer) {
  const int fd = tcp_client_open(buffer);
  if (fd < 0)
    return -1;

  const int connect = tcp_client_connect(fd, host, port, buffer);
  if (connect == -1 || connect == 0) {  // Failed to lookup host or connection failed
    tcp_client_close(fd);
    return -1;
  }
  return fd;
}

int tcp_client_open(char *buffer) {
#ifdef _WIN32
  // initialize the socket API if needed
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {  // Winsock 1.1
    snprintf(buffer, ERROR_BUFFER_SIZE, "Cannot initialize Winsock");
    return -1;
  }
#endif
  /* create the socket */
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    snprintf(buffer, ERROR_BUFFER_SIZE, "Cannot create socket");
    return -1;
  }
  return fd;
}

int tcp_client_connect(int fd, const char *host, int port, char *buffer) {
  struct sockaddr_in address;
  const struct hostent *server;
  // fill in the socket address
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  server = gethostbyname(host);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    snprintf(buffer, ERROR_BUFFER_SIZE, "Cannot resolve server name: %s", host);
    return -1;
  }
  /* connect to the server */
  const int rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    snprintf(buffer, ERROR_BUFFER_SIZE, "Cannot connect to OmniSim instance");
    return 0;
  }
  return 1;
}

bool tcp_client_send(int fd, const char *buffer, int size) {
  char *p = (char *)buffer;
  while (size > 0) {
    const int i = send(fd, p, size, 0);
    if (i < 1)
      return false;
    p += i;
    size -= i;
  }
  return true;
}

int tcp_client_receive(int fd, char *buffer, int size) {
  return recv(fd, buffer, size, 0);
}

int tcp_client_receive_timeout(int fd, char *buffer, int size, int timeout_ms) {
  int received = 0;
  int remaining_ms = timeout_ms;
  while (received < size) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    struct timeval tv;
    tv.tv_sec = remaining_ms / 1000;
    tv.tv_usec = (remaining_ms % 1000) * 1000;
    const int ready = select(fd + 1, &set, NULL, NULL, &tv);
    if (ready <= 0)
      return -1;  // timeout or select error
    const int n = recv(fd, buffer + received, size - received, 0);
    if (n <= 0)
      return -1;  // connection closed or error
    received += n;
    remaining_ms = remaining_ms > 10 ? remaining_ms - 10 : 1;
  }
  return received;
}

void tcp_client_close(int fd) {
#ifdef _WIN32
  closesocket(fd);
  if (WSACleanup() != 0)
    fprintf(stderr, "Cannot cleanup Winsock.\n");
#else
  close(fd);
#endif
}
