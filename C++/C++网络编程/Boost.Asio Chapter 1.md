# The Basics

- https://weread.qq.com/web/reader/63232b60723009d66329e16?
- https://mmoaay.gitbooks.io/boost-asio-cpp-network-programming-chinese/content/Chapter1.html

## Creating an endpoint

A typical ***client application,*** before it can communicate with a server application to consume its services, **must obtain the IP address of the host on which the server application is running and the protocol port number associated with it**. A pair of values consisting of an **IP address** and a **protocol port number** that uniquely identifies a particular application running on a particular host in a computer network is called an endpoint.

IP address：

- dot-decimal notation(IPv4)：192.168.10.112
-  hexadecimal notation(IPv6)： FE36::0404:C3FA:EF1E:3829
- DNS name：localhost or www.google.com
-  integer value

The ***server application*** needs to deal with endpoints too. It uses the endpoint to specify to the operating system on which the IP address and protocol port it wants to listen for incoming messages from the clients. The server application usually wants to listen on all IP addresses available on the host. This guarantees that the server application will receive all messages arriving at any IP address and the particular protocol port.

To sum up, the endpoints serve two goals:

- The **client** application uses an endpoint to **designate a particular server application** it wants to communicate with.
- The **server** application uses an endpoint to **specify a local IP address and a port number** on which it wants to receive incoming messages from clients. If there is more than one IP address on the host, the server application will want to **create a special endpoint representing all IP addresses at once**.

This recipe explains how to create endpoints in Boost.Asio both in client and server applications.

## Getting ready

the client application must obtain the raw IP address and the protocol port number designating the server it will communicate with.

The server application on the other hand, as it usually listens for incoming messages on all IP addresses, **only needs to obtain a port number** on which to listen.

## How to do it 

The first one demonstrates **how the client application can create an endpoint to specify the server it wants to communicate with**. 

The second one demonstrates **how the server application creates an endpoint to specify on which IP addresses and port it wants to listen for incoming messages from clients**.

## Creating an endpoint in the client to designate the server

Obtain the server application's IP address and port number. The IP address should be specified as a string in the dot-decimal (IPv4) 

Represent the raw IP address as an object of the `asio::ip::address` class.Instantiate the object of the `asio::ip::tcp::endpoint` class from the address object created in step 2 and a port number.

> **A  raw IP address** refers to the numerical representation of an Internet Protocol (IP) address without any additional formatting or interpretation. An IP address is a unique identifier assigned to each device connected to a computer network that uses the Internet Protocol for communication.
>
> An IP address is typically written in a dotted-decimal format, such as "192.168.0.1". Each segment of the address represents an 8-bit binary value, ranging from 0 to 255, which corresponds to a decimal value.
>
> In addition to the decimal format, an IP address can also be represented in hexadecimal format or binary format. The raw IP address would simply consist of the numerical values without any separators or symbols.
>
> For example, a raw IP address in decimal format like "192.168.0.1" would be represented as 11000000 10101000 00000000 00000001 in binary or C0 A8 00 01 in hexadecimal.
>
> Raw IP addresses are often used in network programming or configuration settings where direct manipulation of the numerical values is necessary. However, in most practical applications, IP addresses are represented and manipulated in their standard dotted-decimal format for readability and ease of use.

```cpp
#include <boost/asio.hpp>
#include <iostream>

int main()
{
  // Step 1. Assume that the client application has already 
  // obtained the IP-address and the protocol port number.
  std::string raw_ip_address = "127.0.0.1";
  unsigned short port_num = 3333;

  // Used to store information about error that happens
  // while parsing the raw IP-address.
  boost::system::error_code ec;
    
  // Step 2. Using IP protocol version independent address
  // representation.
  boost::asio::ip::address ip_address =
    boost::asio::ip::address::from_string(raw_ip_address, ec);

  if (ec.value() != 0) {
    // Provided IP address is invalid. Breaking execution.
    std::cout 
      << "Failed to parse the IP address. Error code = "
      << ec.value() << ". Message: " << ec.message();
      return ec.value();
  }

  // Step 3.
  boost::asio::ip::tcp::endpoint ep(ip_address, port_num);

  // Step 4. The endpoint is ready and can be used to specify a 
  // particular server in the network the client wants to 
  // communicate with.
  
  return 0;
}
```

## Creating the server endpiont 

Obtain the protocol port number on which the server will listen for incoming requests.

> 协议端口号是用于标识网络通信中不同应用程序或服务的特定端口。它是一个数字，范围从0到65535。
>
> 在计算机网络中，使用传输控制协议（TCP）或用户数据报协议（UDP）进行通信。每个协议都使用端口号来将数据传递给正确的应用程序或服务。
>
> 端口号被分为三个范围：
>
> 1. 知名端口（Well-known Ports）：范围从0到1023，用于一些广泛使用的标准服务。例如，HTTP使用端口号80，HTTPS使用端口号443，FTP使用端口号21等。
> 2. 注册端口（Registered Ports）：范围从1024到49151，用于已注册的应用程序或服务。这些端口号分配给特定应用程序或服务的厂商，并记录在IANA（Internet Assigned Numbers Authority）的服务标识符（Service Name and Transport Protocol Port Number Registry）中。
> 3. 动态或私有端口（Dynamic or Private Ports）：范围从49152到65535，可以被动态地分配给客户端应用程序。这些端口号通常用于临时连接，不属于特定的应用程序或服务。
>
> 需要注意的是，使用特定协议时，同一个端口号可以同时用于TCP和UDP协议，也可以有不同的端口号用于不同的协议。
>
> 选择正确的协议端口号对于确保应用程序或服务能够正确通信非常重要，因此在开发网络应用程序或进行网络配置时，了解和正确使用协议端口号是必要

```cpp
#include <boost/asio.hpp>
#include <iostream>

int main()
{
  // Step 1. Here we assume that the server application has
  //already obtained the protocol port number.
  unsigned short port_num = 3333;

  // Step 2. Create special object of asio::ip::address class
  // that specifies all IP-addresses available on the host. Note
  // that here we assume that server works over IPv6 protocol.
  boost::asio::ip::address ip_address = asio::ip::address_v6::any();

  // Step 3.
  boost::asio::ip::tcp::endpoint ep(ip_address, port_num);

  // Step 4. The endpoint is created and can be used to 
  // specify the IP addresses and a port number on which 
  // the server application wants to listen for incoming 
  // connections.

  return 0;
}
```

## How it works...

Let's consider the first code sample. The algorithm it implements is applicable in an application playing a role of a client that is an application that actively initiates the communication session with a server. The client application needs to be provided an IP address and a protocol port number of the server. Here we assume that those values have already been obtained and are available at the beginning of the algorithm, which makes step 1 details a given.

Having obtained the raw IP address, the client application must represent it in terms of the Boost.Asio type system. Boost.Asio provides three classes used to represent an IP address:

- `asio::ip::address_v4`: This represents an IPv4 
- `addressasio::ip::address_v6`: This represents an IPv6 
- `addressasio::ip::address`: This IP-protocol-version-agnostic class can represent both IPv4 and IPv6 addresses

In step 2, we use the `asio::ip::address` class's static method——`from_string()`. This method accepts a raw IP address represented as a string, parses and validates the string, instantiates an object of the `asio::ip::address` class, and returns it to the caller. This method has four overloads. In our sample we use this one:

```cpp
static asio::ip::address from_string(
    const std::string & str,
    boost::system::error_code & ec);
```

This method is very useful as **it checks whether the string passed to it as an argument contains a valid IPv4 or IPv6address** and if it does, instantiates a corresponding object.If the address is invalid, the method will designate an error through the second argument. **It means that this function can be used to validate the raw user input**.

In step 3, we instantiate an object of the `boost::asio::ip::tcp::endpoint` class, passing the IP address and a protocol port number to its constructor. Now, the ep object can be used to designate a server application in the Boost.Asio communication related functions.