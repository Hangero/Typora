# PlantUML

## 基本语法

### 注释

```cpp
@startuml no-scale
' 这是注释
Hello <|-- World
@enduml
```

### 标题

```cpp
@startuml
title Hello Title
Hello <|-- World
@enduml

```

![image-20230624111430136](/home/suyu/.config/Typora/typora-user-images/image-20230624111430136.png)

`title` 与 `end title` 之间的输入可以换行

![image-20230624111449744](/home/suyu/.config/Typora/typora-user-images/image-20230624111449744.png)

### 图注

`caption`之后跟的内容显示为图注

```cpp
@startuml
caption 图１
Hello <|-- World
@enduml
```

![image-20230624111537723](/home/suyu/.config/Typora/typora-user-images/image-20230624111537723.png)

### 类图

#### Class

类

```cpp
@startuml
class Hello
class World
@enduml

```

![image-20230624111639737](/home/suyu/.config/Typora/typora-user-images/image-20230624111639737.png)

#### Interface

接口

```cpp
@startuml
interface Hello
interface World
@enduml
```

#### 抽象类

```cpp
@startuml
abstract class Hello
@enduml

```

#### 枚举

```cpp
@startuml
enum HelloWorld {
    ONE
    TWO
    THREE
}
@enduml

```

### 类型关系

UML中类型之间有六大关系：

- 泛化（Generalization）
- 实现（Realization）
- 关联（Association)
- 聚合（Aggregation）
- 组合(Composition)
- 依赖(Dependency)

接下来逐一说明：

#### 泛化

泛化关系就是类的继承

```cpp
@startuml
Child --|> Parent
Parent2 <|-- Child2
@enduml

```

![image-20230624112423510](/home/suyu/.config/Typora/typora-user-images/image-20230624112423510.png)

#### 实现

```cpp
@startuml
Plane ..|> Flyable
Flyable <|.. Plane
@enduml

```

![image-20230624112517297](/home/suyu/.config/Typora/typora-user-images/image-20230624112517297.png)



#### 依赖

依赖表示使用关系，java中, 被依赖的对象/类, 以方法参数, 局部变量和静态方法调用的形式出现。比如, 厨师在烹饪的时候看了一眼菜谱, 厨师"使用"了菜谱, 照着它炒完菜后，这种使用关系就结束了(临时性).

```cpp
@startuml
Chef ..> Recipe
@enduml

```

![image-20230624113042075](/home/suyu/.config/Typora/typora-user-images/image-20230624113042075.png)



#### 关联

关联关系，表示"拥有"。 相比依赖关系的临时性和单向性，关联关系具有长期性、平等性(可双向)，所以关联表示的关系比依赖更强。比如现实生活中的夫妻, 师生等关系。长期存在并且是相互的关系。 此外关联可以表示一对一，一对多，多对一，多对多等各种关系。

因为比依赖关系更强, 所以是实线+箭头。 双向关联可以省略箭头。

后面两种关系 “聚合” 和 “组合”，都属于关联关系， 用来表示关联关系中整体与部分的关系。java 中 一个 Class 与其成员变量 Class 类型之间就是这种整体与部分的关联关系。

```
@startuml
Address <-- Husband
Husband <--> Wife
Husband2 -- Wife2
@enduml

```

![image-20230624113123256](/home/suyu/.config/Typora/typora-user-images/image-20230624113123256.png)



#### 聚合

聚合关系相对于组合弱一些，整体与部分是可分离的。 比如部门与员工，部门有许多员工，员工离职了部门仍然存在，不受影响。反之部门解散了，员工可以去其他部门(整体与部分可分离)

```cpp
@startuml
Department o-- Employee
@enduml

```

![image-20230624113646984](/home/suyu/.config/Typora/typora-user-images/image-20230624113646984.png)

#### 组合

组合关系中，整体与部分是不可分离的，整体与部分的生命周期保持一致，少了对方自己的存在无意义。例如人体是有四肢组成的，四肢不能脱离人体存在，人体少了四肢也难言完整

```cpp
@startuml
Body "1" *-- "2" Arm
Body "1" *-- "2" Leg
@enduml

```

![image-20230624141257577](/home/suyu/.config/Typora/typora-user-images/image-20230624141257577.png)

|              | 继承         | 实现          | 依赖                             | 关联                     | 聚合                        | 组合                             |
| ------------ | ------------ | ------------- | -------------------------------- | ------------------------ | --------------------------- | -------------------------------- |
| **关系含义** | 功能扩展     | 功能实现      | 使用                             | 拥有                     | 整体-部分（has-a）          | 整体-部分（contains-a）          |
| **关系特征** | -            | -             | 临时性，单向性                   | 长期性，可双向（平等性） | 整体与部分可分离            | 整体与部分不可分离，生命周期一致 |
| **JAVA语法** | extends      | implements    | 方法参数，局部变量，静态方法调用 | 成员变量                 | 成员变量                    | 成员变量                         |
| **关系强弱** | 强           | 强            | 弱                               | 较强                     | 较强                        | 非常强                           |
| **显示事例** | 父子         | 飞机/鸟可以飞 | 厨师使用菜谱                     | 夫妻，师生               | 部门-员工                   | 人体-四肢                        |
| **图形指向** | 箭头指向父类 | 箭头指向接口  | 箭头指向被使用者                 | 指向被拥有者，可双向     | 箭头指向部分， 菱形指向整体 | 箭头指向部分，菱形指向整体       |

### 成员变量，成员方法

```cpp
@startuml
class Hello {
    one: String
    three(param1: String, param2: int): boolean
    String two
    int four(List<String> param)
}
@enduml

```

![image-20230624182857402](/home/suyu/.config/Typora/typora-user-images/image-20230624182857402.png)

#### 成员的可见性

| Character | Visibility        |
| --------- | ----------------- |
| `-`       | `private`         |
| `#`       | `protected`       |
| `~`       | `package private` |
| `+`       | `public`          |

但是 PlantUML 将这种文字符合进一步图形化：

```cpp
@startuml
class Hello {
    - privateField: int
    # protectedField: int
    ~ packagePrivateField: int
    + publicField: int

    - privateMethod(): void
    # protectedMethod(): void
    ~ packagePrivateMethod(): void
    + publicMethod(): void
}
@enduml

```

![image-20230624183115580](/home/suyu/.config/Typora/typora-user-images/image-20230624183115580.png)

也可以关闭这种图形化符合，继续使用文字符号

通过 `skinparam classAttributeIconSize 0` 关闭图形化符号

```cpp
@startuml
skinparam classAttributeIconSize 0
class Hello {
    - privateField: int
    # protectedField: int
    ~ packagePrivateField: int
    + publicField: int

    - privateMethod(): void
    # protectedMethod(): void
    ~ packagePrivateMethod(): void
    + publicMethod(): void
}
@enduml

```

![image-20230624183145021](/home/suyu/.config/Typora/typora-user-images/image-20230624183145021.png)

#### 抽象方法

```cpp
@startuml
class Hello {
    {abstract} one: int
    {abstract} two(): int
}
@enduml

```

![image-20230624185010490](/home/suyu/.config/Typora/typora-user-images/image-20230624185010490.png)

#### 静态方法

```cpp
@startuml
class Hello {
    {static} ONE: int
    {static} two(): int
}
@enduml

```

![image-20230624185046921](/home/suyu/.config/Typora/typora-user-images/image-20230624185046921.png)

#### 泛型

```cpp
@startuml
class Hello<H>
class World<W> 
@enduml

```

![image-20230624185119174](/home/suyu/.config/Typora/typora-user-images/image-20230624185119174.png)

### 包图

```cpp
@startuml
package one.two {
    class Hello
}

package three.four {
    World -- Hello
}
@enduml

```

![image-20230624185245545](/home/suyu/.config/Typora/typora-user-images/image-20230624185245545.png)

#### 包图中的声明顺序

```cpp
@startuml
package three.four {
    World -- Hello
}

package one.two {
    class Hello
}
@enduml

```

![image-20230624185400276](/home/suyu/.config/Typora/typora-user-images/image-20230624185400276.png)

包图的顺序很重要，如上图 `one.two` 中的类被 `three.four` 依赖，所以应该写到先面， 以为 `Hello` 会声明在第一个出现的包中。

### 备注

使用 `note <top|bottom|left|right>: <备注>` 为 UML 图添加备注， 备注内容可以是 Creole 语法

```cpp
@startuml
class Fizz
note left: fizz

class Buzz
note right: buzz

class Foo
note top: foo

class Bar
note bottom: bar
@enduml

```

![image-20230624185455324](/home/suyu/.config/Typora/typora-user-images/image-20230624185455324.png)

#### 指定目标类

```cpp
@startuml
Fizz -- Buzz
note left of Fizz: fizz
note right of Buzz: buzz
@enduml

```

`note <位置> of <目标>: <备注>`用来为指定目标 Class 生成备注

#### 为类关系进行备注

`note on link: <备注>` 可以在类图的关系中添加备注

```cpp
@startuml
Fizz -- Buzz
note on link: fizz-buzz
note left: buzz
@enduml

```

![image-20230624185709580](/home/suyu/.config/Typora/typora-user-images/image-20230624185709580.png)

#### 给备注添加名字

`note "<备注>" as <名字>`用来给备注设置名字，有了名字后，可以通过名字将一个备注关联到多个Class

```cpp
@startuml
note "Hello World" as n1
Hello -- n1
World .. n1

note "Fizz Buzz" as n2
@enduml

```

![image-20230624185740613](/home/suyu/.config/Typora/typora-user-images/image-20230624185740613.png)

#### 多行备注

`end note` 用来结束多行的备注

```cpp
@startuml
class Hello
note left
Hello
World
end note

Fizz -- Buzz
note on link
Fizz
Buzz
end note
note left of Fizz
fizz
buzz
end note

note as n1
Foo
Bar
end note
@enduml

```

![image-20230624185828438](/home/suyu/.config/Typora/typora-user-images/image-20230624185828438.png)

## 序列图

https://plantuml.com/zh/sequence-diagram