# Boost.Asio C++

Sockets API has several flaws. First,because it was designed as a very generic API that should support many different protocols, it is quite complex and somewhat difficult to use. The second flaw is that this is a C-style functional API with a poor type system, which makes it error prone and even more difficult to use. For example, Sockets API doesn't provide a separate type representing a socket。 Instead, the built-in type int is used, which means that by mistake any value of the int type can be passed as an argument to the function expecting a socket, and the compiler won't detect the mistake. This may lead to run-time crashes, the root cause of which is hard to find.

Network programming is inherently complex and doing it with a low-level C-style socket API makes it even more complex and error prone.

 **Boost.Asio is an O-O C++ library that is, just like raw Sockets API,** built around the concept of a socket.Roughly speaking, Boost.Asio wraps raw Sockets API and provides the developer with O-O interface to it. It is intended to simplify network programming in several ways as follows:

It hides the raw C-style API and providing a user with an object-oriented APIIt provides a rich-type system, which makes code more readable and allows it to catch many errors at compilation timeAs Boost.Asio is a cross-platform library, it simplifies development of cross-platform distributed applicationsIt provides auxiliary functionality such as scatter-gather I/O operations, stream-based I/O, exception-based error handling, and others

The library is designed so that it can be relatively easily extended to add new custom functionalityThis chapter introduces essential Boost.Asio classes and demonstrates how to perform basic operations with them.

## 