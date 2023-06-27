# Lua

[Learn X in Y minutes](https://learnxinyminutes.com/docs/lua/)

## 简介

Lua是一种**轻量级的、可嵌入的、高效的脚本编程语言**。它具有简单的语法和动态类型系统，并且被设计成可扩展的，可以用于嵌入其他程序中作为脚本语言使用，也可以作为独立的解释器运行。

Lua最初在1993年由巴西里约热内卢天主教大学（Pontifical Catholic University of Rio de Janeiro）的一个研究小组开发，其目标是为巴西的电子邮件系统提供一个简单、可扩展的脚本语言。Lua的名字在葡萄牙语中意味着“月亮”。

Lua具有许多特点，使其成为在各种环境中使用的流行脚本语言之一。它的语法简单而灵活，**具有过程式编程和函数式编程的特性**。它支持动态类型、自动内存管理和垃圾回收，使其在资源受限的环境中表现出色。Lua还具有强大的表达式处理能力和元表（metatables）的概念，允许进行元编程和创建自定义数据结构。

Lua广泛应用于游戏开发、嵌入式系统、网络应用、脚本扩展等领域。它被许多知名游戏引擎（如Unity和Corona SDK）采用作为脚本语言，用于编写游戏逻辑和可扩展性。Lua也被用作配置文件和插件脚本语言，以及一些高性能的网络应用程序中的嵌入式脚本语言。

## Learn Lua in Y minutes

```lua
-- Two dashes start a one-line comment. (两个破折号单行注释)
--[[
     Adding two ['s and ]'s makes it a
     multi-line comment.
	 进行多行注释 [[]]
--]]

```

### 变量和流控制

```lua
----------------------------------------------------
-- 1. Variables and flow control.
----------------------------------------------------
num = 42  -- All numbers are doubles.

-- strings
s = 'walternate'  
t = "double-quotes are also fine"
u = [[ Double brackets
       start and end
       multi-line strings.]]
-- Undefines t
t = nil   -- Lua has garbage collection.

-- Blocks are denoted with keywords like do/end: 块用关键字表示
while num < 50 do
  num = num + 1  -- No ++ or += type operators.
end

-- If clauses: If 条款
if num > 40 then
  print('over 40')
elseif s ~= 'walternate' then  -- ~= is not equals.
  -- Equality check is == like Python; ok for strs.
  io.write('not over 40\n')  -- Defaults to stdout.
else
  -- Variables are global by default.
  thisIsGlobal = 5  -- Camel case is common.

  -- How to make a variable local:
  local line = io.read()  -- Reads next stdin line.

  -- String concatenation uses the .. operator:
  print('Winter is coming, ' .. line)
end
```

> Lua中的`print`和`io.write()`都是用于输出信息的函数，但它们之间有一些区别。
>
> 1. `print`函数：`print`是Lua提供的一个全局函数，用于将一个或多个值打印到标准输出（通常是控制台）。它可以接受多个参数，并自动在每个参数之间添加制表符（tab）分隔符，并在最后一个参数后添加换行符。例如：
>
> ```lua
> luaCopy codeprint("Hello", "World")  -- 输出：Hello    World
> print("Lua")             -- 输出：Lua
> ```
>
> **`print`函数的输出默认会转换为字符串形式**，如果传入的参数是表（table）类型，则会打印该表的地址。可以通过自定义元方法来改变表的打印行为。
>
> 1. `io.write()`函数：`io.write()`是Lua标准库中的一个函数，用于将指定的值直接写入文件或输出流，而不会添加额外的分隔符或换行符。它需要显式指定输出对象，通常是一个文件句柄。例如：
>
> ```lua
> luaCopy codelocal file = io.open("output.txt", "w")  -- 打开文件以供写入
> io.write(file, "Hello", "World")        -- 将值写入文件
> io.write(file, "Lua")                   -- 继续写入
> file:close()                            -- 关闭文件
> 
> io.write("Hello", "World")               -- 直接写入到标准输出
> io.write("Lua")                          -- 继续写入
> io.flush()                              -- 刷新输出缓冲
> ```
>
> `io.write()`函数可以通过指定不同的输出对象（如文件句柄）来将值写入不同的目标，例如文件、网络连接等。它也可以用于输出到标准输出（控制台）。
>
> 总的来说，`print`函数更方便用于快速输出调试信息到控制台，而`io.write()`函数更适合将值直接写入指定的输出对象，具有更高的灵活性。

```lua
-- Undefined variables return nil. 未定义的变量返回nil
-- This is not an error:
foo = anUnknownVariable  -- Now foo = nil.

-- Only nil and false are falsy; 0 and '' are true! 只有nil和false为错
aBoolValue = false
if not aBoolValue then print('it was false') end

-- 'or' and 'and' are short-circuited.  or 和 and 具有短路求值（short-circuit evaluation）的特性
-- This is similar to the a?b:c operator in C/js:
ans = aBoolValue and 'yes' or 'no'  --> 'no'

```

> 
> 在Lua中，逻辑运算符 `or` 和 `and` 具有短路求值（short-circuit evaluation）的特性。
>
> 短路求值意味着在执行逻辑运算时，当确定整个表达式的结果已经确定时，就停止对后续表达式的求值，节省了不必要的计算。
>
> - `or` 运算符的短路求值：如果第一个表达式的值为真（非`nil`或`false`），则整个表达式的结果就已经确定为真，不再对第二个表达式进行求值；如果第一个表达式的值为假（`nil`或`false`），则整个表达式的结果由第二个表达式决定。示例：
>
> ```lua
> luaCopy codelocal a = 5
> local b = nil
> 
> local result = a or b
> print(result)  -- 输出：5
> 
> result = b or a
> print(result)  -- 输出：5
> ```
>
> 在第一个示例中，因为变量 `a` 的值为真（非`nil`或`false`），所以整个表达式的结果就是 `a` 的值，不再对变量 `b` 进行求值。
>
> - `and` 运算符的短路求值：如果第一个表达式的值为假（`nil`或`false`），则整个表达式的结果就已经确定为假，不再对第二个表达式进行求值；如果第一个表达式的值为真（非`nil`或`false`），则整个表达式的结果由第二个表达式决定。示例：
>
> ```lua
> luaCopy codelocal a = 5
> local b = nil
> 
> local result = a and b
> print(result)  -- 输出：nil
> 
> result = b and a
> print(result)  -- 输出：nil
> ```
>
> 在第一个示例中，因为变量 `a` 的值为真（非`nil`或`false`），所以整个表达式的结果由变量 `b` 决定，而 `b` 的值为 `nil`，所以最终结果为 `nil`。
>
> 短路求值可以在逻辑表达式中用于简化条件判断，并且可以避免不必要的计算，提高代码的效率。但需要注意，在使用短路求值时，需要确保表达式的顺序和求值结果的预期一致，避免出现意外的结果。

```lua
karlSum = 0
for i = 1, 100 do  -- The range includes both ends.
  karlSum = karlSum + i
end


karlSum = 0
for i = 1, 100 do  -- The range includes both ends.
  karlSum = karlSum + i
end

-- Use "100, 1, -1" as the range to count down:
fredSum = 0
for j = 100, 1, -1 do fredSum = fredSum + j end

-- In general, the range is begin, end[, step]. 开始，结尾，步长

-- Another loop construct:
repeat
  print('the way of the future')
  num = num - 1
until num == 0
```

### 函数

```lua
----------------------------------------------------
-- 2. Functions.
----------------------------------------------------

function fib(n)
  if n < 2 then return 1 end
  return fib(n - 2) + fib(n - 1)
end

-- Closures and anonymous functions are ok:
function adder(x)
  -- The returned function is created when adder is
  -- called, and remembers the value of x:
  return function (y) return x + y end
end
a1 = adder(9)
a2 = adder(36)
print(a1(16))  --> 25
print(a2(64))  --> 100

```

> 这段代码展示了一个闭包（closure）的例子。
>
> 在代码中，`adder` 函数接受一个参数 `x`，然后返回一个函数。返回的函数形成了一个闭包，它记住了在 `adder` 被调用时传入的 `x` 的值。
>
> 返回的函数可以通过调用 `a1` 和 `a2` 来使用。当调用 `a1` 时，它会返回 `x + y` 的结果，其中 `x` 的值是之前传入 `adder` 的 `9`，`y` 的值是调用 `a1` 时传入的参数 `16`。同理，当调用 `a2` 时，它会返回 `x + y` 的结果，其中 `x` 的值是之前传入 `adder` 的 `36`，`y` 的值是调用 `a2` 时传入的参数 `64`。
>
> 因此，代码中的 `print(a1(16))` 打印出 `25`，而 `print(a2(64))` 打印出 `100`。
>
> 闭包是一种函数和其相关环境的组合，它可以访问定义它时所在的词法作用域中的变量。这使得闭包非常灵活，可以用于创建状态保持的函数或实现类似函数工厂的功能。在这个例子中，通过闭包，每个返回的函数都记住了其对应的 `x` 值，使得每次调用时可以使用不同的 `x` 值进行计算。

```lua

```

