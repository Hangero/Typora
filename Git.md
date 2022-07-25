# Git

## 创建版本库

### git init 

```shell
git init
```

### git add & git commit

```shell
$ git add   xxx.txt

$ git commit -m "wrote a readme file"
```

因为`commit`可以一次提交很多文件，所以你可以多次`add`不同的文件，比如：

```shell
$ git add file1.txt
$ git add file2.txt file3.txt
$ git commit -m "add 3 files."
```

## 版本管理

### 版本回退

#### git log

在Git中，我们用`git log`命令查看历史记录，可以用`--pretty=oneline`筛选信息。

类似`1094adb...`的是`commit id`（版本号）

在Git中，用`HEAD`表示当前版本，也就是最新的提交，上一个版本就是`HEAD^`，上上一个版本就是`HEAD^^`，甚至`HEAD~100`。

#### git reset

```shell
$  git reset --hard HEAD^
$  git reset --hard 1094a#输入版本号前几位
```

#### git reflog

Git提供了一个命令`git reflog`用来记录每一次命令

*工作区和暂存区*

- 工作区（Working Directory），就是电脑里能看到的目录
-  版本库（Repository），工作区有一个隐藏目录`.git`，不算工作区，而是Git的版本库。Git的版本库里存了很多东西，其中最重要的就是称为stage（或者叫index）的暂存区，还有Git为我们自动创建的第一个分支`master`，以及指向`master`的一个指针叫`HEAD`。

第一步是用`git add`把文件添加进去，实际上就是把文件修改添加到暂存区；

第二步是用`git commit`提交更改，实际上就是把暂存区的所有内容提交到当前分支。

#### git status

查看状态

### 管理修改

Git跟踪并管理的是修改，而非文件。

假设操作：

第一次修改 -> `git add` -> 第二次修改 -> `git commit`

用`git add`命令后，在工作区的第一次修改被放入暂存区，准备提交，但是，在工作区的第二次修改并没有放入暂存区，所以，`git commit`只负责把暂存区的修改提交了，也就是第一次的修改被提交，第二次的修改不会被提交。

#### [git diff](https://blog.csdn.net/liuxiao723846/article/details/109689069?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165866191916781432952422%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165866191916781432952422&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-1-109689069-null-null.142^v33^experiment_28w_v1,185^v2^control&utm_term=git%20diff%20&spm=1018.2226.3001.4187)

### 撤销修改

#### git checkout -- \<file>

命令`git checkout -- readme.txt`意思就是，把`readme.txt`文件在工作区的修改全部撤销，这里有两种情况：

- 若`readme.txt`自修改后还没有被放到暂存区，现在，撤销修改就回到和版本库一模一样的状态；

- 若`readme.txt`已经添加到暂存区后，又作了修改，现在，撤销修改就回到添加到暂存区后的状态。

总之，就是让这个文件回到最近一次`git commit`或`git add`时的状态，直接丢弃之前的修改。

注意，`git checkout -- file`命令中的`--`很重要，没有`--`，含义为“切换到另一个分支”。

#### git reset HEAD \<file>

`git reset HEAD <file>`可以把暂存区的修改撤销掉（unstage），重新放回工作区。`git reset`命令既可以回退版本，也可以把暂存区的修改回退到工作区。当我们用`HEAD`时，表示最新的版本

场景1：当你改乱了工作区某个文件的内容，想直接丢弃工作区的修改时，用命令`git checkout -- file`。

场景2：当你不但改乱了工作区某个文件的内容，还添加到了暂存区时，想丢弃修改，分两步，第一步用命令`git reset HEAD <file>`，就回到了场景1，第二步按场景1操作。

场景3：已经提交了不合适的修改到版本库时，想要撤销本次提交，参考版本回退一节，不过前提是没有推送到远程库。

### 删除文件

#### git rm

```shell
$  rm test.txt
$  git rm test.txt
```

`git checkout`其实是用版本库里的版本替换工作区的版本，无论工作区是修改还是删除，都可以“一键还原”。

## 分支管理

### 创建与合并分支

一开始的时候，`master`分支是一条线，Git用`master`指向最新的提交，再用`HEAD`指向`master`，就能确定当前分支，以及当前分支的提交点：

![](/home/suyu/Typora/image/Git/0.png)

每次提交，`master`分支都会向前移动一步，这样，随着你不断提交，`master`分支的线也越来越长。

当我们创建新的分支，例如`dev`时，Git新建了一个指针叫`dev`，指向`master`相同的提交，再把`HEAD`指向`dev`，就表示当前分支在`dev`上：

![](/home/suyu/Typora/image/Git/l.png)

从现在开始，对工作区的修改和提交就是针对`dev`分支了，比如新提交一次后，`dev`指针往前移动一步，而`master`指针不变：

![](/home/suyu/Typora/image/Git/l (2).png)

假如我们在`dev`上的工作完成了，就可以把`dev`合并到`master`上。Git怎么合并呢？最简单的方法，就是直接把`master`指向`dev`的当前提交，就完成了合并：

![](/home/suyu/Typora/image/Git/0 (1).png)

合并完分支后，甚至可以删除`dev`分支。删除`dev`分支就是把`dev`指针给删掉，删掉后，我们就剩下了一条`master`分支

![](/home/suyu/Typora/image/Git/0 (2).png)

#### git checkout

```shell
git checkout -b dev
#git checkout命令加上-b参数表示创建并切换，相当于以下两条命令：
git branch dev
git checkout dev
```

#### git branch

`git branch`命令会列出所有分支，当前分支前面会标一个`*`号。

#### git merge

`git merge`命令用于**合并指定分支到当前分支**，一定要注意当前所属分支。

注意，`Fast-forward`信息，Git告诉我们，这次合并是“快进模式”，也就是直接把`master`指向`dev`的当前提交，所以合并速度非常快。

[合并策略](https://blog.csdn.net/qq_27706119/article/details/120557395?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165867961816781683911786%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165867961816781683911786&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-120557395-null-null.142^v33^experiment_28w_v1,185^v2^control&utm_term=git%E5%90%88%E5%B9%B6%E6%A8%A1%E5%BC%8F&spm=1018.2226.3001.4187)

因为创建、合并和删除分支非常快，所以Git鼓励使用分支完成某个任务，合并后再删掉分支，这和直接在`master`分支上工作效果是一样的，但过程更安全。

#### git switch

```shell
$  git switch -c dev#创建并切换到新的dev分支
$  git switch master#直接切换到已有的master分支
```

### 解决冲突

![](/home/suyu/Typora/image/Git/0 (3).png)

这种情况下，Git无法执行“快速合并”，只能试图把各自的修改合并起来，但这种合并就可能会有冲突。

```shell
git log --graph --pretty=oneline --abbrev-commit
```

### 分支管理

#### git merge --no-ff -m 

通常，合并分支时，如果可能，Git会用`Fast forward`模式，但这种模式下，删除分支后，会丢掉分支信息。

如果要强制禁用`Fast forward`模式，Git就会在merge时生成一个新的commit，这样，从分支历史上就可以看出分支信息。

```shell
git merge --no-ff -m "merge with no-ff" dev#--no-ff参数，表示禁用Fast forward
```

不使用`Fast forward`模式，merge后就像这样：

![](/home/suyu/Typora/image/Git/0 (4).png)

![](/home/suyu/Typora/image/Git/0 (5).png)

合并分支时，加上`--no-ff`参数就可以用普通模式合并，合并后的历史有分支，能看出来曾经做过合并，而`fast forward`合并就看不出来曾经做过合并。

### BUG分支

#### git stash

可以把当前工作现场“储藏”起来，等以后恢复现场后继续工作。

git stash list

恢复现场：

- `git stash apply`恢复，但是恢复后，stash内容并不删除，你需要用`git stash drop`来删除

- `git stash pop`，恢复的同时把stash内容也删了

#### git cherry-pick

Git专门提供了一个`cherry-pick`命令，让我们能**复制一个特定的提交**到当前分支

### Feature分支

```shell
git branch -d 

git branch -D <name>#强制删除
```

### 多人协作

#### git remote -v

```shell
$ git remote -v
origin  git@github.com:michaelliao/learngit.git (fetch)
origin  git@github.com:michaelliao/learngit.git (push)
```

上面显示了可以抓取和推送的`origin`的地址。如果没有推送权限，就看不到push的地址。

### 推送分支

####  git push

推送分支，就是把该分支上的所有本地提交推送到远程库。推送时，要指定本地分支，这样，Git就会把该分支推送到远程库对应的远程分支上

### 抓取分支

#### git checkout -b dev origin/dev

在本地创建和远程分支对应的分支，使用`git checkout -b branch-name origin/branch-name`，本地和远程分支的名称最好一致；

#### git pull

Git已经提示我们，先用`git pull`把最新的提交从`origin/dev`抓下来，然后，在本地合并，解决冲突，再推送

#### git branch --set-upstream-to

```shell
git branch --set-upstream-to=origin/dev dev
```

如果`git pull`提示`no tracking information`，则说明本地分支和远程分支的链接关系没有创建，用命令`git branch --set-upstream-to <branch-name> origin/<branch-name>`。

设置`dev`和`origin/dev`的链接

多人协作的工作模式通常是这样：

1. 首先，可以试图用`git push origin <branch-name>`推送自己的修改；
2. 如果推送失败，则因为远程分支比你的本地更新，需要先用`git pull`试图合并；
3. 如果合并有冲突，则解决冲突，并在本地提交；
4. 没有冲突或者解决掉冲突后，再用`git push origin <branch-name>`推送就能成功！

### Rebase



## 标签管理

发布一个版本时，我们通常先在版本库中打一个标签（tag），这样，就唯一确定了打标签时刻的版本。将来无论什么时候，取某个标签的版本，就是把那个打标签的时刻的历史版本取出来。所以，标签也是版本库的一个快照

### 创建标签

#### git tag

`git tag <name>`就可以打一个新标签，默认标签是打在最新提交的commit上的。

```shell
$ git tag v0.9 f52c633
```

还可以创建带有说明的标签，用`-a`指定标签名，`-m`指定说明文字

```shell
$ git tag -a v0.1 -m "version 0.1 released" 1094adb
```



#### git show

标签不是按时间顺序列出，而是按字母排序的。可以用`git show <tagname>`查看标签信息：

标签总是和某个commit挂钩。如果这个commit既出现在master分支，又出现在dev分支，那么在这两个分支上都可以看到这个标签。

### 操作标签

```shell
$ git tag -d v0.1#删除

$ git push origin <tagname>#推送某个标签到远程

$ git push origin --tags#一次性推送全部尚未推送到远程的本地标签

$ git tag -d v0.9
$ git push origin :refs/tags/v0.9#从远程删除
```

