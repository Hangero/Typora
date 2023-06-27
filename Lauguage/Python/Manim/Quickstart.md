# Quickstart

## Overview

First, you will use a command line interface to create a `Scene`, the class through which Manim generates videos. In the `Scene` you will animate a circle. 

Then you will add another `Scene` showing a square transforming into a circle. This will be your introduction to Manim’s animation ability.

 Afterwards, you will position multiple mathematical objects (`Mobject`s). Finally, you will learn the `.animate` syntax, a powerful feature that animates the methods you use to modify `Mobject`s.

## Starting a new project

Start by creating a new folder. For the purposes of this guide, name the folder `project`:

```python
project/
```

This folder is the root folder for your project. It contains all the files that Manim needs to function, as well as any output that your project produces.

## Animating a circle

1. Open a text editor, such as Notepad. Copy the following code snippet into the window:

```python
from manim import *


class CreateCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set the color and transparency
        self.play(Create(circle))  # show the circle on screen
```

2. Save the code snippet into your project folder with the name `scene.py`.

   ```shell
   project/
   └─scene.py
   ```

3. Open the command line, navigate to your project folder, and execute the following command:

   ```shell
   manim -pql scene.py CreateCircle
   ```

> `-pql`（或者`--preview`, `--quality`, `--low_quality`）: 指定渲染器的质量。在这个例子中，使用`-pql`参数将使用预览质量的渲染器，以获得更快的渲染速度和较低的图像质量。
>
> `CreateCircle`: 这是在`scene.py`文件中定义的场景类的名称。Manim将执行该类中定义的动画。
>
> 当使用Manim命令行进行更高质量的渲染、设置分辨率和调整动画持续时间时，可以使用以下参数：
>
> 1. 渲染质量参数：
>
>    - `-p`（或者`--preview`）: 使用预览质量的渲染器，具有较低的图像质量和较快的渲染速度。
>    - `-s`（或者`--medium_quality`）: 使用中等质量的渲染器，默认质量设置。
>    - `-q`（或者`--high_quality`）: 使用高质量的渲染器，具有更好的图像质量和较慢的渲染速度。
>
>    例如，使用`-q`参数可以提高渲染质量：
>
>    ```
>    cssCopy code
>    manim -q scene.py MyScene
>    ```
>
> 2. 分辨率参数：
>
>    - `-r`（或者`--resolution`）: 设置渲染的分辨率，可以指定为`WIDTHxHEIGHT`的形式，例如`-r 1280x720`。
>
>    例如，使用`-r`参数设置分辨率为1280x720：
>
>    ```
>    Copy code
>    manim -pql -r 1280x720 scene.py MyScene
>    ```
>
> 3. 动画持续时间参数：
>
>    - `-d`（或者`--duration`）: 设置动画的持续时间，以秒为单位。
>
>    例如，使用`-d`参数设置动画持续时间为3秒：
>
>    ```
>    Copy code
>    manim -pql -d 3 scene.py MyScene
>    ```
>
> 这些参数可以根据你的需求进行组合和调整。请记住，Manim具有许多其他命令行选项和参数，你可以查阅官方的Manim文档以获取更多信息，并根据需要进行调整。

### Explanation

The first line imports all of the contents of the library:

```
from manim import *
```



This is the recommended way of using Manim, as a single script often uses multiple names from the Manim namespace. In your script, you imported and used `Scene`, `Circle`, `PINK` and `Create`.

Now let’s look at the next two lines:

```python
class CreateCircle(Scene):
    def construct(self):
        ...
```



Most of the time, the code for scripting an animation is entirely contained within the [`construct()`](https://docs.manim.community/en/stable/reference/manim.scene.scene.Scene.html#manim.scene.scene.Scene.construct) method of a [`Scene`](https://docs.manim.community/en/stable/reference/manim.scene.scene.Scene.html#manim.scene.scene.Scene) class. Inside [`construct()`](https://docs.manim.community/en/stable/reference/manim.scene.scene.Scene.html#manim.scene.scene.Scene.construct), you can create objects, display them on screen, and animate them.

The next two lines create a circle and set its color and opacity:

```python
circle = Circle()  # create a circle
circle.set_fill(PINK, opacity=0.5)  # set the color and transparency
```



Finally, the last line uses the animation [`Create`](https://docs.manim.community/en/stable/reference/manim.animation.creation.Create.html#manim.animation.creation.Create) to display the circle on your screen:

```python
self.play(Create(circle))  # show the circle on screen
```

> **All animations must reside within the `construct()`method of a class derived from `Scene`.** Other code, such as auxiliary or mathematical functions, may reside outside the class.

## Transforming a square into a circle

```python
class SquareToCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set color and transparency

        square = Square()  # create a square
        square.rotate(PI / 4)  # rotate a certain amount

        self.play(Create(square))  # animate the creation of the square
        self.play(Transform(square, circle))  # interpolate the square into the circle
        self.play(FadeOut(square))  # fade out animation
```

Render `SquareToCircle` by running the following command in the command line:

```python
manim -pql scene.py SquareToCircle
```

## Positoning `Mobject`s

```python
class SquareAndCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set the color and transparency

        square = Square()  # create a square
        square.set_fill(BLUE, opacity=0.5)  # set the color and transparency

        square.next_to(circle, RIGHT, buff=0.5)  # set the position
        self.play(Create(circle), Create(square))  # show the shapes on screen
```

`next_to` is a `Mobject` method for positioning `Mobject`s.

## Using `.animate` syntax to animate methods

The final lesson in this tutorial is using `.animate`, a `Mobject` method which animates changes you make to a `Mobject`. When you prepend `.animate` to any method call that modifies a `Mobject`, the method becomes an animation which can be played using `self.play`. Let’s return to `SquareToCircle` to see the differences between using methods when creating a `Mobject`, and animating those method calls with `.animate`.

1. Open `scene.py`, and add the following code snippet below the `SquareAndCircle` class:

   ```python
   class AnimatedSquareToCircle(Scene):
       def construct(self):
           circle = Circle()  # create a circle
           square = Square()  # create a square
   
           self.play(Create(square))  # show the square on screen
           self.play(square.animate.rotate(PI / 4))  # rotate the square
           self.play(
               ReplacementTransform(square, circle)
           )  # transform the square into a circle
           self.play(
               circle.animate.set_fill(PINK, opacity=0.5)
           )  # color the circle on screen
   ```

2. Render `AnimatedSquareToCircle` by running the following command in the command line:

   ```shell
   manim -pql scene.py AnimatedSquareToCircle
   ```

3. Open `scene.py`, and add the following code snippet below the `AnimatedSquareToCircle` class:

   ```python
   class DifferentRotations(Scene):
       def construct(self):
           left_square = Square(color=BLUE, fill_opacity=0.7).shift(2 * LEFT)
           right_square = Square(color=GREEN, fill_opacity=0.7).shift(2 * RIGHT)
           self.play(
               left_square.animate.rotate(PI), Rotate(right_square, angle=PI), run_time=2
           )
           self.wait()
   ```

4. Render `DifferentRotations` by running the following command in the command line:

   ```
   manim -pql scene.py DifferentRotations
   ```

   

​		































