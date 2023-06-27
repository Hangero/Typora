# Portobuf

## 初识

### 是什么

`Protocol buffers(协议缓冲器) `是一种灵活，高效，自动化机制的结构数据序列化方法——可类比 XML，但是比 XML 更小、更快、更为简单。你可以定义数据的结构，然后使用特殊生成的源代码轻松的在各种数据流中使用各种语言进行编写和读取结构数据。你甚至可以更新数据结构，而不破坏根据旧数据结构编译而成并且已部署的程序。

### 如何工作

你可以通过在 .proto 文件中定义 protocol [buffer](https://so.csdn.net/so/search?q=buffer&spm=1001.2101.3001.7020) message 类型，来指定你想如何对序列化信息进行结构化。每一个 protocol buffer message 是一个信息的小逻辑记录，包含了一系列的 name-value 对。这里有一个非常基础的 .proto 文件样例，它定义了一个包含 "person" 相关信息的 message：

```protobuf
message Person {
  required string name = 1;
  required int32 id = 2;
  optional string email = 3;

  enum PhoneType {
    MOBILE = 0;
    HOME = 1;
    WORK = 2;
  }

  message PhoneNumber {
    required string number = 1;
    optional PhoneType type = 2 [default = HOME];
  }

  repeated PhoneNumber phone = 4;
}
```

正如你所见，message 格式很简单 ——每种 message 类型都有一个或多个具有唯一编号的字段，每个字段都有一个名称和一个值类型；其中值类型可以是数字（整数或浮点数），布尔值，字符串，原始字节，甚至（如上例所示）其它 protocol buffer message 类型，这意味着允许你分层次地构建数据。 你可以在 [Protocol Buffer 语言指南](https://www.jianshu.com/p/6f68fb2c7d19) 中找到有关编写 `.proto` 文件的更多信息

一旦定义了 messages，就可以在 .proto 文件上运行 protocol buffer 编译器来生成指定语言的数据访问类。这些类为每个字段提供了简单的访问器（如 name()和 set_name()），以及将整个结构序列化为原始字节和解析原始字节的方法 - 例如，如果你选择的语言是 C++，则运行编译器上面的例子将**生成一个名为 Person 的类**。然后，你可以在应用程序中使用此类来填充，序列化和检索 Person 的 messages。于是你可以写一些这样的代码：

```cpp
Person person;
person.set_name("John Doe");
person.set_id(1234);
person.set_email("jdoe@example.com");
fstream output("myfile", ios::out | ios::binary);
person.SerializeToOstream(&output);
```

之后，你可以重新读取解析你的 message

```cpp
fstream input("myfile", ios::in | ios::binary);
Person person;
person.ParseFromIstream(&input);
cout << "Name: " << person.name() << endl;
cout << "E-mail: " << person.email() << endl;
```

你**可以在 message 格式中添加新字段，而不会破坏向后兼容性**；旧的二进制文件在解析时只是忽略新字段。因此，如果你的通信协议使用 protocol buffers 作为其数据格式，则可以扩展协议而无需担心破坏现有代码。 你可以在 [API 参考部分](https://developers.google.com/protocol-buffers/docs/reference/overview) 中找到使用生成的 protocol buffer 代码的完整参考，你可以在 [协议缓冲区编码](https://www.jianshu.com/p/82ff31c6adc6) 中找到更多关于如何对 protocol buffer messages 进行编码的信息。

### 为什么不使用XML

对于序列化结构数据，protocol buffers 比 XML 更具优势。Protocol buffers：

- 更简单
- 小 3 ~ 10 倍
- 快 20 ~ 100 倍
- 更加清晰明确
- 自动生成更易于以编程方式使用的数据访问类

例如，假设你想要为具有姓名和电子邮件的人建模。在XML中，你需要：

```html
<person>
	<name>John Doe</name>
	<email>jdoe@example.com</email>
</person>
```

而相对应的 protocol buffer message（参见 protocol buffer [文本格式](https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.text_format)）是：

```html
# Textual representation of a protocol buffer.
# This is *not* the binary format used on the wire.
person {
	name: "John Doe"
	email: "jdoe@example.com"
}
```

当此消息被编码为 protocol buffer [二进制格式](https://www.jianshu.com/p/82ff31c6adc6) 时（**上面的文本格式只是为了调试和编辑的方便而用人类可读的形式表示**），它可能是 28 个字节长，需要大约 100-200 纳秒来解析。如果删除空格，XML版本至少为 69 个字节，并且需要大约 5,000-10,000 纳秒才能解析。 此外，比起 XML，操作 protocol buffer 更为容易：

```html
cout << "Name: " << person.name() << endl;
cout << "E-mail: " << person.email() << endl;
```

而使用 XML，你必须执行以下操作：

```html
cout << "Name: "
	<< person.getElementsByTagName("name")->item(0)->innerText()
	<< endl;
cout << "E-mail: "
	<< person.getElementsByTagName("email")->item(0)->innerText()
	<< endl;
```

但是，protocol buffers 并不总是比 XML 更好的解决方案 ，例如：

- protocol buffers 不是使用标记（例如 HTML）对基于文本的文档建模的好方法，因为你无法轻松地将结构与文本交错。
- XML 是人类可读的和人类可编辑的；protocol buffers，至少它们的原生格式，并不具有这样的特点。
- XML 在某种程度上也是自我描述的。只有拥有 message 定义（.proto文件）时，protocol buffer 才有意义。

## 语言指导

### 定义一个消息类型

我们想定义个关于搜索请求的消息，每个搜索请求包含一个查询字符串，一个特定的页码，和每页的结果数量。下面是用于定义消息类型的 `.proto` 文件：

```protobuf
syntax = "proto3";

message SearchRequest {
  string query = 1;
  int32 page_number = 2;
  int32 result_per_page = 3;
}
```

- 文件的第一行指明了我们使用的是 proto3 语法：若不指定该行 protocol buffer 编译器会认为是 proto2 。**该行必须是文件的第一个非空或非注释行**。
- `SearchRequest` 消息定义了三个字段（名称/值对），字段就是每个要包含在该类型消息中的部分数据。每个字段都具有名称和类型 。

#### 指定字段类型

上面的例子中，全部字段都是标量类型：两个整型（`page_number` 和 `result_per_page`）和一个字符串型（`query`）。同样，也可以指定复合类型的字段，包括枚举型和其他消息类型。

#### 分配字段编号

正如你所见，消息中定义的每个字段都有一个**唯一编号**。字段编号用于在消息二进制格式中标识字段，同时**要求消息一旦使用字段编号就不应该改变**。注意一点 1 到 15 的字段编号需要用 1 个字节来编码，编码同时包括字段编号和字段类型（ 获取更多信息请参考 [Protocol Buffer Encoding](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Fencoding.html%23structure) ）。16 到 2047 的字段变化使用 2 个字节。因此应将 1 到 15 的编号用在消息的常用字段上。注意应该为将来可能添加的常用字段预留字段编号。

#### 指定字段规则

（没看懂）

消息的字段可以是一下规则之一：

- singular ， 格式良好的消息可以有 0 个或 1 个该字段(但不能多于 1 个)。这是 proto3 语法的默认字段规则。
- repeated ，格式良好的消息中该字段可以重复任意次数（包括 0 次）。重复值的顺序将被保留。

在 proto3 中，标量数值类型的重复字段默认会使用 packed 压缩编码。

更多关于 packed 压缩编码的信息请参考 [Protocol Buffer Encoding](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Fencoding.html%23packed) 。

#### 增加更多的消息类型

单个 .proto 文件中可以定义多个消息类型。这在定义相关联的多个消息中很有用——例如要定义与搜索消息`SearchRequest` 相对应的回复消息 `SearchResponse`，则可以在同一个 .proto 文件中增加它的定义：

```protobuf
message SearchRequest {
  string query = 1;
  int32 page_number = 2;
  int32 result_per_page = 3;
}

message SearchResponse {
 ...
}
```

#### 添加注释

使用 C/C++ 风格的 `//` 和 `/* ... */` 语法在 .proto 文件添加注释。

```protobuf
/* SearchRequest represents a search query, with pagination options to
 * indicate which results to include in the response. */

message SearchRequest {
  string query = 1;
  int32 page_number = 2;  // Which page number do we want?
  int32 result_per_page = 3;  // Number of results to return per page.
}
```

#### 保留字段

**在采取彻底删除或注释掉某个字段的方式来更新消息类型时，将来其他用户再更新该消息类型时可能会重用这个字段编号。**后面再加载该 .ptoto 的旧版本时会引发好多问题，例如数据损坏，隐私漏洞等。**一个防止该问题发生的办法是将删除字段的编号**（或字段名称，字段名称会导致在 JSON 序列化时产生问题）**设置为保留项 `reserved`**。protocol buffer 编译器在用户使用这些保留字段时会发出警告。

```protobuf
message Foo {
  reserved 2, 15, 9 to 11;
  reserved "foo", "bar";
}
```

注意，**不能在同一条 `reserved` 语句中同时使用字段编号和名称**。

#### `.proto`文件会生成什么？

当 protocol buffer 编译器作用于一个 .proto 文件时，编辑器会生成基于所选编程语言的关于 .proto 文件中描述消息类型的相关代码 ，包括对字段值的获取和设置，序列化消息用于输出流，和从输入流解析消息。

- 对于 **C++**， 编辑器会针对于每个 `.proto` 文件生成`.h` 和 `.cc` 文件，对于每个消息类型会生成一个类。
- 对于 **Java**, 编译器会生成一个 `.java` 文件和每个消息类型对应的类，同时包含一个特定的 `Builder`类用于构建消息实例。
- **Python** 有些不同 – Python 编译器会对于 .proto 文件中每个消息类型生成一个带有静态描述符的模块，以便于在运行时使用 metaclass 来创建必要的 Python 数据访问类。

### 标量数据类型

消息标量字段可以是以下类型之一——下表列出了可以用在 .proto 文件中使用的类型，以及在生成代码中的相关类型：

| .proto Type | Notes                                                        | C++ Type | Java Type  | Python Type[2] |
| :---------- | :----------------------------------------------------------- | :------- | :--------- | :------------- |
| double      |                                                              | double   | double     | float          |
| float       |                                                              | float    | float      | float          |
| int32       | 使用变长编码。负数的编码效率较低——若字段可能为负值，应使用 sint32 代替。 | int32    | int        | int            |
| int64       | 使用变长编码。负数的编码效率较低——若字段可能为负值，应使用 sint64 代替。 | int64    | long       | int/long[3]    |
| uint32      | 使用变长编码。                                               | uint32   | int[1]     | int/long[3]    |
| uint64      | 使用变长编码。                                               | uint64   | long[1]    | int/long[3]    |
| sint32      | 使用变长编码。符号整型。负值的编码效率高于常规的 int32 类型。 | int32    | int        | int            |
| sint64      | 使用变长编码。符号整型。负值的编码效率高于常规的 int64 类型。 | int64    | long       | int/long[3]    |
| fixed32     | 定长 4 字节。若值常大于2^28 则会比 uint32 更高效。           | uint32   | int[1]     | int/long[3]    |
| fixed64     | 定长 8 字节。若值常大于2^56 则会比 uint64 更高效。           | uint64   | long[1]    | int/long[3]    |
| sfixed32    | 定长 4 字节。                                                | int32    | int        | int            |
| sfixed64    | 定长 8 字节。                                                | int64    | long       | int/long[3]    |
| bool        |                                                              | bool     | boolean    | bool           |
| string      | 包含 UTF-8 和 ASCII 编码的字符串，长度不能超过 2^32 。       | string   | String     | str/unicode[4] |
| bytes       | 可包含任意的字节序列但长度不能超过 2^32 。                   | string   | ByteString | str            |

可以在 [Protocol Buffer Encoding](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Fencoding) 中获取更多关于消息序列化时类型编码的相关信息。

> 1.  Java 中，无符号 32 位和 64 位整数使用它们对应的符号整数表示，第一个 bit 位仅是简单地存储在符号位中。
> 2. 所有情况下，设置字段的值将执行类型检查以确保其有效。
> 3. 64 位或无符号 32 位整数在解码时始终表示为 long，但如果在设置字段时给出 int，则可以为 int。在所有情况下，该值必须适合设置时的类型。见 2。
> 4. Python 字符串在解码时表示为 unicode，但如果给出了 ASCII 字符串，则可以是 str（这条可能会发生变化）。

### 默认值

当解析消息时，若消息编码中没有包含某个元素，则相应的会使用该字段的默认值。默认值依据类型而不同：

- 字符串类型，空字符串
- 字节类型，空字节
- 布尔类型，false
- 数值类型，0
- 枚举类型，第一个枚举元素
- 内嵌消息类型，依赖于所使用的编程语言。

对于可重复类型字段的默认值是空的（ 通常是相应语言的一个空列表 ）。

注意一下标量字段，在消息被解析后是不能区分字段是使用默认值（例如一个布尔型字段是否被设置为 false ）赋值还是被设置为某个值的。例如你不能通过对布尔值等于 false 的判断来执行一个不希望在默认情况下执行的行为。同时还要注意若一个标量字段设置为默认的值，那么是不会被序列化以用于传输的。

查看 [generated code guide](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Foverview) 来获得更多关于编程语言生成代码的内容。

### 枚举

定义消息类型时，可能需要某字段值是一些预设值之一。例如当需要在 `SearchRequest` 消息类型中增加一个 `corpus` 字段， `corpus` 字段的值可以是 `UNIVERSAL`， `WEB`，`IMAGES`， `LOCAL`， `NEWS`， `PRODUCTS` 或 `VIDEO`。仅仅需要在消息类型中定义带有预设值常量的 `enum` 类型即可完成上面的定义。

```cpp
message SearchRequest {
  string query = 1;
  int32 page_number = 2;
  int32 result_per_page = 3;
  enum Corpus {
    UNIVERSAL = 0;
    WEB = 1;
    IMAGES = 2;
    LOCAL = 3;
    NEWS = 4;
    PRODUCTS = 5;
    VIDEO = 6;
  }
  Corpus corpus = 4;
}
```

`Corpus` 枚举类型的第一个常量映射到 0 ：每个枚举的定义必须包含一个映射到 0 的常量作为第一个元素。原因是：

- 必须有一个 0 值，才可以作为数值类型的默认值。
- 0 值常量必须作为第一个元素，是为了与 proto2 的语义兼容，就是第一个元素作为默认值。

**将相同的枚举值分配给不同的枚举选项常量可以定义别名**。要定义别名需要将 `allow_alisa` 选项设置为 `true`，否则 protocol 编译器当发现别名定义时会报错。

```cpp
enum EnumAllowingAlias {
  option allow_alias = true;
  UNKNOWN = 0;
  STARTED = 1;
  RUNNING = 1;
}
enum EnumNotAllowingAlias {
  UNKNOWN = 0;
  STARTED = 1;
  // RUNNING = 1;  // Uncommenting this line will cause a compile error inside Google and a warning message outside.
}
```

#### 保留值

在采取彻底删除或注释掉某个枚举值的方式来更新枚举类型时，将来其他用户再更新该枚举类型时可能会重用这个枚举数值。后面再加载该 .ptoto 的旧版本时会引发好多问题，例如数据损坏，隐私漏洞等。一个防止该问题发生的办法是将删除的枚举数值（或名称，名称会导致在 JSON 序列化时产生问题）设置为保留项 `reserved`。protocol buffer 编译器在用户使用这些特定数值时会发出警告。可以使用 `max` 关键字来指定保留值的范围到最大可能值。

```cpp
enum Foo {
  reserved 2, 15, 9 to 11, 40 to max;
  reserved "FOO", "BAR";
}
```

### 使用其他消息类型

消息类型也可作为字段类型。例如，我们需要在 `SearchResponse` 消息中包含 `Result` 消息——想要做到这一点，可以将 `Result` 消息类型的定义放在同一个 .proto 文件中同时在 `SearchResponse` 消息中指定一个 `Result` 类型的字段：

```protobuf
message SearchResponse {
  repeated Result results = 1;
}

message Result {
  string url = 1;
  string title = 2;
  repeated string snippets = 3;
}   
```

#### 导入定义

前面的例子中，我们将 `Result` 消息定义在了与 `SearchResponse` 相同的文件中——但若我们需要作为字段类型使用的消息类型已经定义在其他的 .proto 文件中了呢？

可以通过导入操作来使用定义在其他 .proto 文件中的消息定义。在文件的顶部使用 import 语句完成导入其他 .proto 文件中的定义：

```protobuf
import "myproject/other_protos.proto";
```

默认情况下仅可以通过直接导入 .proto 文件来使用这些定义。然而有时会需要将 .proto 文件移动位置。可以通过在原始位置放置一个伪 .proto 文件使用 `import public` 概念来转发对新位置的导入，而不是在发生一点更改时就去更新全部对旧文件的导入位置。任何导入包含 `import public` 语句的 proto 文件就会对其中的 `import public` 依赖产生传递依赖。例如：

```protobuf
// new.proto
// 全部定义移动到该文件

// old.proto
// 这是在客户端中导入的伪文件
import public "new.proto";
import "other.proto";

// client.proto
import "old.proto";
// 可使用 old.proto 和 new.proto 中的定义，但不能使用 other.proto 中的定义
```

protocol 编译器会使用命令行参数 `-I`/`--proto_path` 所指定的目录集合中检索需要导入的文件。若没有指定，会在调用编译器的目录中检索。通常应该将 `--proto_path` 设置为项目的根目录同时在 import 语句中使用全限定名。

### 嵌套类型

可以在一个消息类型中定义和使用另一个消息类型，如下例所示—— `Result` 消息类型定义在了 `SearchResponse` 消息类型中：

```protobuf
message SearchResponse {
  message Result {
    string url = 1;
    string title = 2;
    repeated string snippets = 3;
  }
  repeated Result results = 1;
}
```

使用 `Parent.Type` 语法可以在父级消息类型外重用内部定义消息类型：

```protobuf
message SomeOtherMessage {
  SearchResponse.Result result = 1;
}
```

支持任意深度的嵌套：

```protobuf
message Outer {                  // Level 0
  message MiddleAA {  // Level 1
    message Inner {   // Level 2
      int64 ival = 1;
      bool  booly = 2;
    }
  }
  message MiddleBB {  // Level 1
    message Inner {   // Level 2
      int32 ival = 1;
      bool  booly = 2;
    }
  }
}
```

### 消息类型的更新

如果现有的消息类型不再满足您的所有需求——例如，需要扩展一个字段——同时还要继续使用已有代码，别慌！ 在不破坏任何现有代码的情况下更新消息类型非常简单。仅仅遵循如下规则即可：

- **不要修改任何已有字段的字段编号**
- 若是添加新字段，旧代码序列化的消息仍然可以被新代码所解析。应该牢记新元素的默认值以便于新代码与旧代码序列化的消息进行交互。类似的，新代码序列化的消息同样可以被旧代码解析：旧代码解析时会简单的略过新字段。参考未知字段获取详细信息。
- 字段可被移除，只要不再使用移除字段的字段编号即可。可能还会对字段进行重命名，或许是增加前缀 `OBSOLETE_` ，或保留字段编号以保证后续不能重用该编号。
- `int32`， `uint32`， `int64`， `uint64`， 和 `bool` 是完全兼容的——意味着可以从这些字段其中的一个更改为另一个而不破坏前后兼容性。若解析出来的数值与相应的类型不匹配，会采用与 C++ 一致的处理方案（例如，若将 64 位整数当做 32 位进行读取，则会被转换为 32 位）。
- `sint32` 和`sint64` 相互兼容但不与其他的整型兼容。
- `string` and `bytes` 在合法 UTF-8 字节前提下也是兼容的。
- 嵌套消息与 `bytes` 在 bytes 包含消息编码版本的情况下也是兼容的。
- `fixed32` 与`sfixed32` 兼容， `fixed64` 与 `sfixed64`兼容。
- `enum` 与 `int32`，`uint32`， `int64`，和 `uint64` 兼容（注意若值不匹配会被截断）。但要注意当客户端反序列化消息时会采用不同的处理方案：例如，未识别的 proto3 枚举类型会被保存在消息中，但是当消息反序列化时如何表示是依赖于编程语言的。整型字段总是会保持其的值。
- **将一个单独值更改为新 `oneof` 类型成员之一是安全和二进制兼容的。** 若确定没有代码一次性设置多个值那么将多个字段移入一个新 `oneof` 类型也是可行的。将任何字段移入已存在的 `oneof` 类型是不安全的。

### 未知字段

未知字段是解析结构良好的 protocol buffer 已序列化数据中的未识别字段的表示方式。例如，当旧程序解析带有新字段的数据时，这些新字段就会成为旧程序的未知字段。

本来，proto3 在解析消息时总是会丢弃未知字段，但在 3.5 版本中重新引入了对未知字段的保留机制以用来兼容 proto2 的行为。在 3.5 或更高版本中，未知字段在解析时会被保留同时也会包含在序列化结果中。

### Any类型

Any 类型允许我们将没有 .proto 定义的消息作为内嵌类型来使用。一个 `Any` 包含一个类似 `bytes` 的任意序列化消息，以及一个 URL（Uniform Resource Locator,统一资源定位器） 来作为消息类型的全局唯一标识符。要使用 `Any` 类型，需要导入 `google/protobuf/any.proto`。

```protobuf
import "google/protobuf/any.proto";

message ErrorStatus {
  string message = 1;
  repeated google.protobuf.Any details = 2;
}
```

不同的语言实现会支持运行时的助手函数来完成类型安全地 `Any` 值的打包和拆包工作——C++ 中是 `PackFrom()` 和 `UnpackTo()` 方法：

```protobuf
// Storing an arbitrary message type in Any.
NetworkErrorDetails details = ...;
ErrorStatus status;
status.add_details()->PackFrom(details);
​
// Reading an arbitrary message from Any.
ErrorStatus status = ...;
for (const Any& detail : status.details()) {
  if (detail.Is<NetworkErrorDetails>()) {
    NetworkErrorDetails network_error;
    detail.UnpackTo(&network_error);
    ... processing network_error ...
  }
}
```

### 包

可以在 .proto 文件中使用 `package` 指示符来避免 protocol 消息类型间的命名冲突。

```protobuf
package foo.bar;
message Open { ... }
```

这样在定义消息的字段类型时就可以使用包指示符来完成：

```protobuf
message Foo {
  ...
  foo.bar.Open open = 1;
  ...
}
```

包指示符的处理方式是基于编程语言的：

- C++ 中生成的类位于命名空间中。例如，`Open` 会位于命名空间 `foo::bar` 中。

- Python 中，package 指示符被忽略，这是因为 Python 的模块是基于文件系统的位置来组织的。

  protocol buffer 中类型名称解析的工作机制类似于 C++ ：先搜索最内层作用域，然后是次内层，以此类推，每个包被认为是其外部包的内层。前导点（例如，`.foo.bar.Baz`）表示从最外层作用域开始。

  protocol buffer 编译器会解析导入的 .proto 文件中的全部类型名称。基于编程语言生成的代码也知道如何去引用每种类型，即使编程语言有不同的作用域规则。

### 定义服务

若要在 RPC （Remote Procedure Call，远程过程调用）系统中使用我们定义的消息类型，则可在 .proto 文件中定义这个 RPC 服务接口，同时 protocol buffer 编译器会基于所选编程语言生成该服务接口代码。例如，若需要定义一个含有可以接收 `SearchRequest` 消息并返回 `SearchResponse` 消息方法的 RPC 服务，可以在 .proto 文件中使用如下代码定义：

```protobuf
service SearchService {
  rpc Search (SearchRequest) returns (SearchResponse);
}
```

### 选项

.proto 文件中的单个声明可以被一组选项来设置。选项不是用来更改声明的含义，但会影响在特定上下文下的处理方式。完整的选项列表定义在 `google/protobuf/descriptor.proto` 中。

选项有以下级别：

- 文件级：意味着可以卸载顶级作用域，而不是在消息、枚举、或服务的定义中。
- 消息级：意味着需写在消息的定义中。
- 字段级：意味着需要写在字段的定义内。

- `optimize_for （文件选项）`： 可被设为 `SPEED`， `CODE_SIZE`，或 `LITE_RUNTIME`。这会影响 C++ 和 Java 代码生成器（可能包含第三方生成器） 的以下几个方面：
  - `SPEED （默认）`： protocol buffer 编译器将生成用于序列化、解析和消息类型常用操作的代码。生成的代码是高度优化的。
  - `CODE_SIZE `：protocol buffer 编译器将生成最小化的类，并依赖于共享的、基于反射的代码来实现序列化、解析和各种其他操作。因此，生成的代码将比 SPEED 模式小的多，但操作将变慢。类仍将实现与 SPEED 模式相同的公共 API。这种模式在处理包含大量 .proto 文件同时不需要所有操作都要求速度的应用程序中最有用。
  - `LITE_RUNTIME` ：protocol buffer 编译器将生成仅依赖于 “lite” 运行库的类（libprotobuf-lite 而不是libprotobuf）。lite 运行时比完整的库小得多（大约小一个数量级），但会忽略某些特性，比如描述符和反射。这对于在受限平台（如移动电话）上运行的应用程序尤其有用。编译器仍然会像在 SPEED 模式下那样生成所有方法的快速实现。生成的类将仅用每种语言实现 MessageLite 接口，该接口只提供 `Message` 接口的一个子集。

```html
option optimize_for = CODE_SIZE;    
```

- `cc_enable_arenas`（文件选项）：为生成的 C++ 代码启用 [arena allocation](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Farenas) 。
- `objc_class_prefix` （文件选项）： 设置当前 .proto 文件生成的 Objective-C 类和枚举的前缀。没有默认值。你应该使用 [recommended by Apple](https://links.jianshu.com/go?to=https%3A%2F%2Fdeveloper.apple.com%2Flibrary%2Fios%2Fdocumentation%2FCocoa%2FConceptual%2FProgrammingWithObjectiveC%2FConventions%2FConventions.html%23%2F%2Fapple_ref%2Fdoc%2Fuid%2FTP40011210-CH10-SW4) 的 3-5 个大写字母作为前缀。注意所有 2 个字母前缀都由 Apple 保留。

### 调用protobuf编译器

Protocol buffer 编译器的调用方式如下:

```protobuf
protoc --proto_path=IMPORT_PATH --cpp_out=DST_DIR python_out=DST_DIR 
```

`IMPORT_PATH` 为`import` 指令检索 .proto 文件的目录。若未指定，使用当前目录。多个导入目录可以通过多次传递 `--proto_path` 选项实现；这些目录会依顺序检索。 `-I=*IMPORT_PATH*` 可作为 `--proto_path` 的简易格式使用。

可以提供一个或多个输出指令：

- `--cpp_out` 在 `DST_DIR`目录 生成 C++ 代码。参阅 [C++ generated code reference](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp-generated) 获取更多信息。

- `--python_out`在 `DST_DIR`目录 生成 Python代码。参阅 [Python generated code reference](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fpython-generated) 获取更多信息。

## 规范引导

### Message 和字段命名

使用驼峰命名法（首字母大写）命名 message，例子：**SongServerRequest** 使用下划线命名字段，栗子：**song_name**

```protobuf
message SongServerRequest {
  required string song_name = 1;
}
```

使用上述这种字段的命名约定，生成的访问器将类似于如下代码：

```cpp
//C++:
  const string& song_name() { ... }
  void set_song_name(const string& x) { ... }

```

### 枚举enums

使用驼峰命名法（首字母大写）命名枚举类型，使用 “大写*下划线*大写” 的方式命名枚举值：

```protobuf
enum Foo {
 FIRST_VALUE = 0;
 SECOND_VALUE = 1;
}
```

每一个枚举值以分号结尾，而非逗号。

### 服务Services

如果你在 .proto 文件中定义 RPC 服务，你应该使用驼峰命名法（首字母大写）命名 RPC 服务以及其中的 RPC 方法：

```protobuf
service FooService {
  rpc GetSomething(FooRequest) returns (FooResponse);
}
```

## 编码

[protobuf介绍](https://blog.csdn.net/sinat_35945236/article/details/114611396?spm=1001.2014.3001.5506#t37)

## 相关技术

### 多消息（Message）流

如果要将多条消息（Message）写入单个文件或流，则需要跟踪一条消息的结束位置和下一条消息的开始位置。因为 Protocol Buffer 数据格式不是自定界限的，因此 Protocol Buffer 解析器无法确定消息自身的结束位置。解决此问题最简单的方法就是在每条消息本身内容之前记录消息的大小或长度。当你重新读取消息时，可读取其大小，读取对应字节到单独的缓冲区，然后从该缓存区中解析消息内容。(如果你不想将字节复制到单独的缓冲区，请查看 CodedInputStream 类（C++ 和 Java 都具有此类），这个类可以限制读入缓冲区的字节数）。

### 大数据集

Protocol Buffers 并不是为处理大信息（large messages）而设计的。依据一般的经验法则，如果你处理的是每个 message 都大于兆字节的数据（messages），那么这个时候可能需要考虑换一种策略。

也就是说，protocol buffers 非常适合处理大数据集中的单个消息。通常大数据集是一些小块数据的集合，而其中每个小块可能是结构化的数据。 尽管 protocol buffers 无法同时处理整个数据集，但可以使用 protocol buffers 对每一小块进行编码从而极大简化我们的问题：现在我们只需要处理一组字节字符串而不是一组结构。

Protocol Buffers 并没有内置对大数据集的支持，因为不同的情况通常需要不同的解决方案。有时一个简单的记录列表就能满足需求，而有时你需要的可能是一个更接近数据库的东西。每一个解决方案都应该作为单独的库去开发，而只有真正需要这些相应解决方案的才需要付出相应成本。

### 自描述信息

Protocol Buffers 并不包含其自身类型的描述。所以如果没有给出定义类型的 .proto 文件，而只有原始信息（raw message）是很难提取任何有用数据的。

> 自描述信息对于反射实现至关重要

然而，值得注意的是 .proto 文件的内容本身实际上也可以使用 protocol buffers 来表达。源码中的 src/google/protobuf/descriptor.proto 定义了所需的相关类型。protoc 命令可以使用 --descriptor_set_out 选项来输出 FileDescriptorSet（此类就表示一组 .proto 文件）。通过这种方式，你可以定义一个自描述的协议消息，如下所示：

```protobuf
message SelfDescribingMessage {
  // Set of .proto files which define the type.
  required FileDescriptorSet proto_files = 1;

  // Name of the message type.  Must be defined by one of the files in
  // proto_files.
  required string type_name = 2;

  // The message data.
  required bytes message_data = 3;
}
```

## Protobuf Buffer Basics : C++

本教程为 C++ 程序员如何使用 protocol buffers 做一个基本介绍。通过创建一个简单的示例应用程序，它向你展示：

- 如何在一个 `.proto` 文件中定义 message
- 如何使用 protocol buffer 编译器
- 如何使用 C++ protocol buffer 的 API 读写 message

这不是一篇通过 C ++ 使用 protocol 的综合指南。如果想获取更详细的参考信息，请参阅 [Protocol Buffer 语法指引](https://www.jianshu.com/p/6f68fb2c7d19)、[C++ API 指引](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp%2Findex.html)、[C++ 生成代码指引](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp-generated) 和 Protocol Buffer 的 [编码指引](https://www.jianshu.com/p/82ff31c6adc6)。

### 为什么要使用Protobuf Buffers?

我们将要使用的示例是一个非常简单的 “地址簿” 应用程序，可以在文件中读写联系人的详细信息。地址簿中的每个人都有姓名、ID、电子邮件地址和联系电话。

你该如何序列化和反序列化如上结构的数据呢？

而 Protocol buffers 是灵活，高效，自动化的解决方案。采用 protocol buffers，你可以写一个 `.proto` 文件描述你想要读取的数据的结构。由此， protocol buffer 编译器将创建一个类，该类使用有效的二进制格式实现 protocol buffer 数据的自动编码和解析。**生成的类为构成 protocol buffer 的字段提供 getter 和 setter，并负责读写 protocol buffer 单元的细节。重要的是，protocol buffer 的格式支持随着时间的推移扩展格式的想法**，使得代码仍然可以读取用旧格式编码的数据。

### 示例代码

示例代码包含在源代码包中的 "examples" 目录下。[下载地址](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Fdownloads.html)

### 定义protocol格式

要创建地址簿应用程序，你需要从 .proto 文件开始。**`.proto `文件中的定义很简单：为要序列化的每个数据结构添加 `message `定义，然后为 `message `中的每个字段指定名称和类型。**下面就是定义相关 message 的 .proto 文件，addressbook.proto。

```protobuf
syntax = "proto2";

package tutorial;

message Person {
  required string name = 1;
  required int32 id = 2;
  optional string email = 3;

  enum PhoneType {
    MOBILE = 0;
    HOME = 1;
    WORK = 2;
  }

  message PhoneNumber {
    required string number = 1;
    optional PhoneType type = 2 [default = HOME];
  }

  repeated PhoneNumber phones = 4;
}

message AddressBook {
  repeated Person people = 1;
}
```

.proto 文件**以 package 声明开头，这有助于防止不同项目之间的命名冲突**。在 C++ 中，生成的类将放在与包名匹配的 namespace （命名空间）中。

接下来，你将看到相关的 message 定义。message 只是包含一组类型字段的集合。许多标准的简单数据类型都可用作字段类型，包括 bool、int32、float、double 和 string。你还可以使用其他 message 类型作为字段类型在消息中添加更多结构 。在上面的示例中，Person 包含 PhoneNumber message ，而 AddressBook 包含 Person message。你甚至可以定义嵌套在其他 message 中的 message 类型。如你所见，PhoneNumber 类型在 Person 中定义。如果你希望其中一个字段具有预定义的值列表中的值，你还可以定义枚举类型 - 此处你指定（枚举）电话号码，它的值可以是 MOBILE，HOME 或 WORK 之一。

每个元素上的 "=1"，"=2" 标记表示该字段在二进制编码中使用的唯一 “标记”。标签号 1-15 比起更大数字需要少一个字节进行编码，因此以此进行优化，你可以决定将这些标签用于常用或重复的元素，**将标记 16 和更高的标记留给不太常用的可选元素**。repeated 字段中的每个元素都需要重新编码 Tag，因此 repeated 字段特别适合使用此优化。

必须使用以下修饰符之一注释每个字段：

- **required**: 必须提供该字段的值，否则该消息将被视为“未初始化”。如果是在调试模式下编译 libprotobuf，则序列化一个未初始化的 message 将将导致断言失败。在优化的构建中，将跳过检查并始终写入消息。但是，解析未初始化的消息将始终失败（通过从解析方法返回 false）。除此之外，required 字段的行为与 optional 字段完全相同。
- **optional**: 可以设置也可以不设置该字段。如果未设置可选字段值，则使用默认值。对于简单类型，你可以指定自己的默认值，就像我们在示例中为电话号码类型所做的那样。否则，使用系统默认值：数字类型为 0，字符串为空字符串，bools 为 false。对于嵌入 message，默认值始终是消息的 “默认实例” 或 “原型”，其中没有设置任何字段。调用访问器以获取尚未显式设置的 optional（或 required）字段的值始终返回该字段的默认值。
- **repeated**: 该字段可以重复任意次数（包括零次）。重复值的顺序将保留在 protocol buffer 中。**可以将 repeated 字段视为动态大小的数组**。

> 建议只使用`optiona`l和`repeated`

不要去寻找类似于类继承的工具（设计），protocol buffers 不会这样做。

### 编译Protocol Buffers

既然你已经有了一个 `.proto` 文件，那么你需要做的下一件事就是生成你需要读写`AddressBook`（以及 `Person` 和 `PhoneNumber` ） message 所需的类。为此，你需要在 `.proto` 上运行 protocol buffer 编译器 `protoc`：

运行编译器，指定源目录（应用程序的源代码所在的位置 - 如果不提供值，则使用当前目录），目标目录（你希望生成代码的目标目录;通常与源目录`$SRC_DIR`相同），以及`.proto`的路径。在这种情况下，你可以执行如下命令：

```html
protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/addressbook.proto
```

因为你需要 C ++ 类，所以使用 `--cpp_out` 选项 - 当然，为其他支持的语言也提供了类似的选项。

这将在指定的目标目录中生成以下文件：

- `addressbook.pb.h`： 类声明的头文件
- `addressbook.pb.cc`：类实现

### The Protocol Buffer API

让我们看看一些生成的代码，看看编译器为你创建了哪些类和函数。如果你查看　addressbook.pb.h，你会发现你在 addressbook.proto 中指定的每条 message 都有一个对应的类。仔细观察 Person 类，你可以看到编译器已为每个字段生成了访问器。例如，对于 name ，id，email 和 phone 字段，你可以使用以下方法：

```cpp
 // required name
  inline bool has_name() const;
  inline void clear_name();
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline ::std::string* mutable_name();
 
  // required id
  inline bool has_id() const;
  inline void clear_id();
  inline int32_t id() const;
  inline void set_id(int32_t value);
 
  // optional email
  inline bool has_email() const;
  inline void clear_email();
  inline const ::std::string& email() const;
  inline void set_email(const ::std::string& value);//?
  inline void set_email(const char* value);
  inline ::std::string* mutable_email();
 
  // repeated phones
  inline int phones_size() const;
  inline void clear_phones();
  inline const ::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >& phones() const;
  inline ::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >* mutable_phones();
  inline const ::tutorial::Person_PhoneNumber& phones(int index) const;
  inline ::tutorial::Person_PhoneNumber* mutable_phones(int index);
  inline ::tutorial::Person_PhoneNumber* add_phones();
```



> [namespace和using的用法](https://blog.csdn.net/qq_64691289/article/details/128339110?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167837670416800182157512%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167837670416800182157512&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-128339110-null-null.142^v73^control,201^v4^add_ask,239^v2^insert_chatgpt&utm_term=c%2B%2B%20namespace%E5%89%8D%E5%8A%A0%3A%3A&spm=1018.2226.3001.4187)

如你所见，getter 的名称与小写字段完全相同，setter 方法以 set_ 开头。每个单数（required 或 optional）字段也有 has_ 方法，如果设置了该字段，则返回 true。最后，每个字段都有一个 clear_ 方法，可以将字段重新设置回 empty 状态。

虽然数字 id 字段只有上面描述的基本访问器集，但是 name 和 email 字段因为是字符串所以有几个额外的方法：一个 mutable_ 的 getter，它允许你获得一个指向字符串的直接指针，以及一个额外的 setter。请注意，即使尚未设置 email ，也可以调用 mutable_email()；它将自动初始化为空字符串。如果在这个例子中你有一个单数的 message 字段，它也会有一个 mutable_ 方法而不是 set_ 方法。

repeated 字段也有一些特殊的方法 - 如果你看一下 repeated phones 字段的相关方法，你会发现你可以：

- 检查 repeated 字段长度（换句话说，与此人关联的电话号码数）
- 使用索引获取指定的电话号码
- 更新指定索引处的现有电话号码
- 在 message 中添加另一个电话号码同时之后也可进行再修改（repeated 的标量类型有一个 add_，而且只允许你传入新值）

有关 protocol 编译器为任何特定字段定义生成的确切成员的详细信息，请参阅 [C++ 生成代码参考](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp-generated)。

### 枚举和嵌套类

生成的代码包含与你的 .proto 枚举对应的 PhoneType 枚举。你**可以将此类型称为 Person::PhoneType，其值为 Person::MOBILE，Person::HOME 和 Person::WORK**（实现细节稍微复杂一些，但你如果仅仅只是使用不需要理解里面的实现原理）。

编译器还为你生成了一个名为 Person::PhoneNumber 的嵌套类。如果查看代码，可以看到 “真实” 类实际上称为 Person_PhoneNumber，但在 Person 中定义的 typedef 允许你将其视为嵌套类。唯一会造成一点差异的情况是，如果你想在另一个文件中前向声明该类 - 你不能在 C ++ 中前向声明嵌套类型，但你可以前向声明 Person_PhoneNumber。

### 标准Message方法

每个 message 类还包含许多其他方法，可用于检查或操作整个 message，包括：

- `bool IsInitialized() const;`: 检查是否已设置所有必填 required 字段
- `string DebugString() const;`: 返回 message 的人类可读表达，对调试特别有用

- `void CopyFrom(const Person& from);`: 用给定的 message 的值覆盖 message
- `void Clear();`: 将所有元素清除回 empty 状态

这些和下一节中描述的 I/O 方法实现了所有 C++ protocol buffer 类共享的 `Message` 接口。更多的更详细的有关信息，请参阅 [Message 的完整 API 文档](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp%2Fgoogle.protobuf.message.html%23Message)。

### 解析和序列化

最后，每个 protocol buffer 类都有使用 protocol buffer [二进制格式](https://www.jianshu.com/p/82ff31c6adc6) 读写所选类型 message 的方法。包括：

- `bool SerializeToString(string* output) const;`:序列化消息并将字节存储在给定的字符串中。请注意，字节是二进制的，而不是文本;我们只使用 `string` 类作为方便的容器。
- `bool ParseFromString(const string& data);`: 解析给定字符串到 message
- `bool SerializeToOstream(ostream* output) const;`: 将 message 写入给定的 C++ 的 ostream
- `bool ParseFromIstream(istream* input);`: 解析给定 C++ istream 到 message

这些只是解析和序列化提供的几个选项。请参阅 [Message API 参考](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp%2Fgoogle.protobuf.message.html%23Message) 以获取完整列表。

> 如果要为生成的类添加更丰富的行为，最好的方法是将生成的 Protocol Buffers 类包装在特定于应用程序的类中。如果你无法控制 .proto 文件的设计（如果你正在重用另一个项目中的一个），那么包装 Protocol Buffers 的类也是一个好主意。在这种情况下，你**可以使用包装器类来创建更适合应用程序的独特环境的接口**：隐藏一些数据和方法，公开便利功能等。**永远不应该通过继承它们来为生成的类添加行为**。这将打破内部机制，无论如何这都不是良好的面向对象的实践。
>
> 译者注：对象模型设计原则之一：**使用组合代替继承**

### 写入一个Message类

现在让我们尝试使用你的 Protocol Buffer 类。你希望地址簿应用程序能够做的第一件事可能是将个人详细信息写入的地址簿文件。为此，你需要创建并填充 Protocol Buffer 类的实例，然后将它们写入输出流。

这是一个从文件中读取 AddressBook 的程序，根据用户输入向其添加一个新 Person，并将新的 AddressBook 重新写回文件。其中直接调用或引用 protocol 编译器生成的代码部分将高亮显示。

> 译者注： “直接调用或引用 protocol 编译器生成的代码部分” 采用注释 ＠＠＠ 的方式标出

```cpp
#include <iostream>
#include <fstream>
#include <string>
#include "addressbook.pb.h"
using namespace std;
 
// This function fills in a Person message based on user input.
void PromptForAddress(tutorial::Person* person) {
  cout << "Enter person ID number: ";
  int id;
  cin >> id;
  person->set_id(id);
  cin.ignore(256, '\n');
 
  cout << "Enter name: ";
  getline(cin, *person->mutable_name());
 
  cout << "Enter email address (blank for none): ";
  string email;
  getline(cin, email);
  if (!email.empty()) {
    person->set_email(email);
  }
 
  while (true) {
    cout << "Enter a phone number (or leave blank to finish): ";
    string number;
    getline(cin, number);
    if (number.empty()) {
      break;
    }
    // ＠＠＠ Person::PhoneNumber
    tutorial::Person::PhoneNumber* phone_number = person->add_phones();
    phone_number->set_number(number);
 
    cout << "Is this a mobile, home, or work phone? ";
    string type;
    getline(cin, type);
    if (type == "mobile") {
      // ＠＠＠ Person
      phone_number->set_type(tutorial::Person::MOBILE);
    } else if (type == "home") {
      // ＠＠＠ Person
      phone_number->set_type(tutorial::Person::HOME);
    } else if (type == "work") {
      // ＠＠＠ Person
      phone_number->set_type(tutorial::Person::WORK);
    } else {
      cout << "Unknown phone type.  Using default." << endl;
    }
  }
}
 
// Main function:  Reads the entire address book from a file,
//   adds one person based on user input, then writes it back out to the same
//   file.
int main(int argc, char* argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
 
  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << " ADDRESS_BOOK_FILE" << endl;
    return -1;
  }
  // ＠＠＠ AddressBook
  tutorial::AddressBook address_book;
 
  {
    // Read the existing address book.
    fstream input(argv[1], ios::in | ios::binary);
    if (!input) {
      cout << argv[1] << ": File not found.  Creating a new file." << endl;
    // ＠＠＠ ParseFromIstream
    } else if (!address_book.ParseFromIstream(&input)) {
      cerr << "Failed to parse address book." << endl;
      return -1;
    }
  }
 
  // Add an address.
  PromptForAddress(address_book.add_people());
 
  {
    // Write the new address book back to disk.
    fstream output(argv[1], ios::out | ios::trunc | ios::binary);
    // ＠＠＠ SerializeToOstream
    if (!address_book.SerializeToOstream(&output)) {
      cerr << "Failed to write address book." << endl;
      return -1;
    }
  }
 
  // Optional:  Delete all global objects allocated by libprotobuf.
  // ＠＠＠ ShutdownProtobufLibrary
  google::protobuf::ShutdownProtobufLibrary();
 
  return 0;
}
```

> - 请注意 GOOGLE_PROTOBUF_VERIFY_VERSION 宏。在使用 C++ Protocol Buffer 库之前执行此宏是一种很好的做法 。它验证你没有意外链接到与你编译的头文件不兼容的库版本。如果检测到版本不匹配，程序将中止。请注意，每个 .pb.cc 文件在启动时都会自动调用此宏。
>
> - **注意在程序结束时调用 ShutdownProtobufLibrary()。**所有这一切都是删除 Protocol Buffer 库分配的所有全局对象。对于大多数程序来说这是不必要的，因为该过程无论如何都要退出，并且操作系统将负责回收其所有内存。但是，如果你使用了内存泄漏检查程序，该程序需要释放每个最后对象，或者你正在编写可以由单个进程多次加载和卸载的库，那么你可能希望强制使用 Protocol Buffers 来清理所有内容。

### 读取一个Message

```cpp
#include <iostream>
#include <fstream>
#include <string>
#include "addressbook.pb.h"
using namespace std;

// Iterates though all people in the AddressBook and prints info about them.
void ListPeople(const tutorial::AddressBook& address_book) {
  for (int i = 0; i < address_book.people_size(); i++) {
    const tutorial::Person& person = address_book.people(i);

    cout << "Person ID: " << person.id() << endl;
    cout << "  Name: " << person.name() << endl;
    if (person.has_email()) {
      cout << "  E-mail address: " << person.email() << endl;
    }

    for (int j = 0; j < person.phones_size(); j++) {
      const tutorial::Person::PhoneNumber& phone_number = person.phones(j);

      switch (phone_number.type()) {
        case tutorial::Person::MOBILE:
          cout << "  Mobile phone #: ";
          break;
        case tutorial::Person::HOME:
          cout << "  Home phone #: ";
          break;
        case tutorial::Person::WORK:
          cout << "  Work phone #: ";
          break;
      }
      cout << phone_number.number() << endl;
    }
  }
}

// Main function:  Reads the entire address book from a file and prints all
//   the information inside.
int main(int argc, char* argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << " ADDRESS_BOOK_FILE" << endl;
    return -1;
  }

  tutorial::AddressBook address_book;

  {
    // Read the existing address book.
    fstream input(argv[1], ios::in | ios::binary);
    if (!address_book.ParseFromIstream(&input)) {
      cerr << "Failed to parse address book." << endl;
      return -1;
    }
  }

  ListPeople(address_book);

  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
```

### 拓展一个Protocol Buffer

在发布使用 protocol buffer 的代码之后，无疑早晚有一天你将会想要 “改进” protocol buffer 的定义。如果你希望你的新 buffer 向后兼容，并且你的旧 buffer 是向前兼容的（实际上你一定想要这种兼容性） - 那么你需要遵循一些规则。在新版本的 protocol buffer 中：

- **你不得更改任何现有字段的字段编号**
- **你不得添加或删除任何 required 字段**
- **你可以删除 optional 或 repeated 的字段**
- **你可以添加新的 optional 或 repeated 字段，但必须使用新的标记号**（即从未在此协议缓冲区中使用的编号，甚至包括那些已删除的字段的编号）

如果你遵循这些规则，旧代码将很乐意阅读新消息并简单地忽略任何新字段。对于旧代码，已删除的可选字段将只具有其默认值，删除的重复字段将为空。新代码也将透明地读取旧消息。但是，**请记住旧的 message 中不会出现新的可选字段，因此你需要明确通过调用 has_ 方法来检查它们是否被设置，或者在字段编号后面使用 [default = value] 在 .proto 文件中提供合理的默认值。**如果未为 optional 元素指定默认值，则使用特定于类型的默认值：对于字符串，默认值为空字符串。对于布尔值，默认值为 false。对于数字类型，默认值为零。另请注意，如果添加了新的 repeated 字段，则新代码将无法判断它是否为空（通过新代码）或从未设置（通过旧代码），因为它没有 has_ 标志。

### 优化技巧

C++ Protocol Buffers 已经做了极大优化。但是，正确使用可以进一步提高性能。以下是压榨最后一点速度的一些提示和技巧：

- **尽可能重用 message 对象。**message 会为了重用尝试保留它们分配的任何内存，即使它们被清除。因此，如果你连续处理具有相同类型和类似结构的许多 message，则每次重新使用相同的 message 对象来加载内存分配器是个好主意。但是，随着时间的推移，对象会变得臃肿，特别是如果你的 message 在 “形状” 上有所不同，或者你偶尔构造的 message 比平时大得多。你应该通过调用 [SpaceUsed](https://links.jianshu.com/go?to=https%3A%2F%2Fdevelopers.google.com%2Fprotocol-buffers%2Fdocs%2Freference%2Fcpp%2Fgoogle.protobuf.message.html%23Message.SpaceUsed.details) 方法来监控邮件对象的大小，一旦它们变得太大就删除它们。
- 你的系统内存分配器可能没有针对从多个线程分配大量小对象这种情况进行良好优化。请尝试使用 [Google 的 tcmalloc](https://links.jianshu.com/go?to=https%3A%2F%2Fgithub.com%2Fgperftools%2Fgperftools)。

# Protobuf API

[Protobuf C++ API 简介](https://blog.csdn.net/qq_22660775/article/details/89193506?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167854232316800225512995%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=167854232316800225512995&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-89193506-null-null.142^v73^control,201^v4^add_ask,239^v2^insert_chatgpt&utm_term=protobuf%20API&spm=1018.2226.3001.4187)





| API                                              | 含义                                         |
| ------------------------------------------------ | -------------------------------------------- |
| string DebugString() const                       | 生成此消息的可读形式，可用于调试和其他目的。 |
| bool SerializeToString(string* output) const;    |                                              |
| bool ParseFromArray(const void * data, int size) | 解析包含在字节数组中的Protobuf               |
|                                                  |                                              |
|                                                  |                                              |
|                                                  |                                              |
|                                                  |                                              |



|      |      |
| ---- | ---- |
|      |      |





[Protobuf发送接收数据类型](https://blog.csdn.net/zxng_work/article/details/78943265?ops_request_misc=&request_id=&biz_id=102&utm_term=protobuf%E6%94%B6%E5%8F%91%E6%95%B0%E6%8D%AE&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-78943265.142^v74^wechat,201^v4^add_ask,239^v2^insert_chatgpt&spm=1018.2226.3001.4187)

## 使用message

```protobuf
//Demo.proto 协议格式文件
syntax='proto3'
package=Demo

message Data {
	optional int32 x = 1;
	optional string str = 2;
	repeated int32 d = 3;
}
```



### 类成员变量的访问

- 获取成员变量：直接采用使用成员变量名（全部为小写）；
- 设置成员变量：使用成员变量名前加`set_`的方法

```
//使用message
#include <Demo.pb.h>
#include <QDebug>

Demo::Data data;
data.set_x(20); //设置成员变量
qDebug()<<data.x(); //获取成员变量
```

> - 限定修饰符 `required` | `repeated`
>
> - 采用以下划线分割的驼峰式，如：first_name 

对于普通成员变量（required)

- 提供`has_`方法判断变量值是否被设置；

- 提供`clear_`方法清除设置的变量值 ；

  ```cpp
  //使用message
  #include <Demo.pb.h>
  #include <QDebug>
  
  Demo::Data data;
  data.set_x(20); //设置成员变量
  qDebug()<<data.has_x(); //判断变量值是否被设置
  data.clear_x(); //清除x设置的变量值
  ```

对于string类型

- 提供了多种`set_`方法，其参数不同；
- **提供了一个`mutable_`方法，返回变量值的可修改指针** ；

```cpp
//使用message
#include <Demo.pb.h>
#include <QDebug>

Demo::Data data;
data.set_str(20); //设置成员变量
std::string* mutable_str(); //返回str变量值的可修改指针

```

对于repeated变量

- `_size`方法：返回变量的长度；
- 通过下脚标访问其中的数据成员组；
- 通过下脚标返回其中的成员的`mutable_`的方法
- `_add`方法：增加一个成员

```cpp
//使用message
#include <Demo.pb.h>
#include <QDebug>

Demo::Data data;
for(int i=0; i<10; i++)
{
    data.d_add(i); //向d中添加成员
}
for(int i=0; i<data.d_size(); i++)
    printf("%d\t",data.d(i)); //通过下脚标访问数据成员组

```

## 序列化与反序列化

参考官方示例实现C++使用protobuf进行序列化和反序列化：

### addressbook.proto :

```protobuf
syntax = "proto3";
package tutorial;

option optimize_for = LITE_RUNTIME;

message Person {
	string name = 1;
	int32 id = 2;
	string email = 3;

	enum PhoneType {
		MOBILE = 0;
		HOME = 1;
		WORK = 2;
	}
	
	message PhoneNumber {
		string number = 1;
		PhoneType type = 2;
	}

	repeated PhoneNumber phones = 4;
}

```

### 生成的addressbook.pb.h 文件内容摘要

```cpp
namespace tutorial {
	class Person;
	class Person_PhoneNumber;
};

class Person_PhoneNumber : public MessageLite {
public:
	Person_PhoneNumber();
	virtual ~Person_PhoneNumber();
public:
	//string number = 1;
	void clear_number();
	const string& number() const;
	void set_number(const string& value);
	
	//int32 id = 2;
	void clear_id();
	int32 id() const;
	void set_id(int32 value);

	//string email = 3; 
	//...
};



```

### add_person.cpp 

```cpp
#include <iostream>
#include <fstream>
#include <string>
#include "pbs/addressbook.pb.h"
using namespace std;

void serialize_process() {
	cout << "serialize_process" << endl;
	tutorial::Person person;
	person.set_name("Obama");
	person.set_id(1234);
	person.set_email("1234@qq.com");

	tutorial::Person::PhoneNumber *phone1 = person.add_phones();
	phone1->set_number("110");
	phone1->set_type(tutorial::Person::MOBILE);

	tutorial::Person::PhoneNumber *phone2 = person.add_phones();
	phone2->set_number("119");
	phone2->set_type(tutorial::Person::HOME);

	fstream output("person_file", ios::out | ios::trunc | ios::binary);

	if( !person.SerializeToOstream(&output) ) {
		cout << "Fail to SerializeToOstream." << endl;
	}

	cout << "person.ByteSizeLong() : " << person.ByteSizLong() << endl;
}

void parse_process() {
	cout << "parse_process" << endl;
	tutorial::Person result;
	fstream input("person_file", ios::in | ios::binary);

	if(!result.ParseFromIstream(&input)) {
		cout << "Fail to ParseFromIstream." << endl;
	}

	cout << result.name() << endl;
	cout << result.id() << endl;
	cout << result.email() << endl;
	for(int i = 0; i < result.phones_size(); ++i) {
		const tutorial::Person::PhoneNumber &person_phone = result.phones(i);

		switch(person_phone.type()) {
			case tutorial::Person::MOBILE :
				cout << "MOBILE phone : ";
				break;
			case tutorial::Person::HOME :
				cout << "HOME phone : ";
				break;
			case tutorial::Person::WORK :
				cout << "WORK phone : ";
				break;
			default:
				cout << "phone type err." << endl;
		}
		cout << person_phone.number() << endl;
	}
}

int main(int argc, char *argv[]) {
	serialize_process();
	parse_process();
	
	google::protobuf::ShutdownProtobufLibrary();	//删除所有已分配的内存（Protobuf使用的堆内存）
	return 0;
}

```

**输出结果**

```shell
[serialize_process]
person.ByteSizeLong() : 39
[parse_process]
Obama
1234
1234@qq.com
MOBILE phone : 110
HOME phone : 119

```

### API接口函数

```cpp
class MessageLite {
public:
	//序列化：
    //C数组
	bool SerializeToArray(void *data, int size) const;
    ///C++ String
	bool SerializeToString(string* output) const;
    ///C++ stream
	bool SerializeToOstream(ostream* output) const;
    ///文件描述符
 	bool SerializeToFileDescriptor(int file_descriptor) const;
	
	//反序列化：
    bool ParseFromArray(const void* data, int size);
	bool ParseFromString(const string& data);
    bool ParseFromIstream(istream* input);
    bool ParseFromFileDescriptor(int file_descriptor);

};

```

#### C 数组

```c
//C数组的序列化和序列化API
bool ParseFromArray(const void* data, int size);
bool SerializeToArray(void* data, int size) const;
//使用
void set_people()             
{
    wp.set_name("sealyao");   
    wp.set_id(123456);        
    wp.set_email("sealyaog@gmail.com");
    wp.SerializeToArray(parray,256);
}
 
void get_people()             
{
    rap.ParseFromArray(parray,256);
    cout << "Get People from Array:" << endl;
    cout << "\t Name : " <<rap.name() << endl;
    cout << "\t Id : " << rap.id() << endl;
    cout << "\t email : " << rap.email() << endl;
}
```

#### C++ String

```cpp
//C++string序列化和序列化API
bool SerializeToString(string* output) const;
bool ParseFromString(const string& data);
//使用：
void set_people()             
{
    wp.set_name("sealyao");   
    wp.set_id(123456);        
    wp.set_email("sealyaog@gmail.com");
    wp.SerializeToString(&pstring);
}
 
void get_people()             
{
    rsp.ParseFromString(pstring);  
    cout << "Get People from String:" << endl;
    cout << "\t Name : " <<rsp.name() << endl;
    cout << "\t Id : " << rsp.id() << endl;
    cout << "\t email : " << rsp.email() << endl;
}
```

#### C++  stream

```c
//C++ stream 序列化/反序列化API
bool SerializeToOstream(ostream* output) const;
bool ParseFromIstream(istream* input);
 
//使用：
void set_people()
{
    fstream fs(path,ios::out|ios::trunc|ios::binary);
    wp.set_name("sealyaog");
    wp.set_id(123456);
    wp.set_email("sealyaog@gmail.com");
    wp.SerializeToOstream(&fs);    
    fs.close();
    fs.clear();
}
 
void get_people()
{
    fstream fs(path,ios::in|ios::binary);
    rp.ParseFromIstream(&fs);
    std::cout << "\t Name : " <<rp.name() << endl;
    std::cout << "\t Id : " << rp.id() << endl; 
    std::cout << "\t email : " << rp.email() << endl;   
    fs.close();
    fs.clear();
}
```

#### 文件描述符

```cpp
 //文件描述符的序列化和序列化API
 bool SerializeToFileDescriptor(int file_descriptor) const;
 bool ParseFromFileDescriptor(int file_descriptor);
 
 //使用:
void set_people()
{
    fd = open(path,O_CREAT|O_TRUNC|O_RDWR,0644);
    if(fd <= 0){
        perror("open");
        exit(0); 
    }   
    wp.set_name("sealyaog");
    wp.set_id(123456);
    wp.set_email("sealyaog@gmail.com");
    wp.SerializeToFileDescriptor(fd);   
    close(fd);
}
 
void get_people()
{
    fd = open(path,O_RDONLY);
    if(fd <= 0){
        perror("open");
        exit(0);
    }
    rp.ParseFromFileDescriptor(fd);
    std::cout << "Get People from FD:" << endl;
    std::cout << "\t Name : " <<rp.name() << endl;
    std::cout << "\t Id : " << rp.id() << endl;
    std::cout << "\t email : " << rp.email() << endl;
    close(fd);
}
```

