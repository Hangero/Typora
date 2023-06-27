# Git使用原则与习惯

[使用git的一些原则、好习惯和常用命令](https://blog.csdn.net/u013553529/article/details/88361442)

## 原则

- 在第一次提交之前，建立`.gitignore`文件，将那些不需要提交的文件忽略掉（例如，密码文件，中间生成的文件和目录，本地设置的文件，临时文件等）

- 一次提交只包含一个相关的改动

  例如，在一次提交中解决一个 bug，或者在一次提交中增加一个新功能。最好不要出现，在一次提交中，既添加了一个新功能，又包含了 bug 修复。这样的好处是，

  1. 在git revert时，可以方便的撤销那个 commit 的改动。
  2. 对开发团队的其他成员来说，更容易理解你做的改动。

- 在提交（`git commit`）之前，一定要经过完整的测试。
- 在本地仓库中，提交到 master 分支后，及时推送（`git push`）到远程仓库中。
- 获取远程代码（`git pull`）之前，最好要查看当前本地仓库的状态`git status`。
- 如果已经把commit 推送到远程仓库服务器了，就不要轻易对本地仓库中的该commit做改动，例如，不要使用`git reset`、`git rebase`、`git commit --amend`对该commit修改。
- 执行 `git reset --hard [commit]`之前，要确保本地工作目录中的文件已经提交到远程仓库，或者已经备份到另一个目录中。**永远要慎重使用这个命令，确保你的辛苦付出不会付之东流。**
- 在执行`git revert`和`git reset`等命令时，一定要清楚当前在哪个分支上。查看当前分支：`git branch -v`。

## 好习惯

- commit message 写的简洁明确，以便其他人明确你的改动意图。

  commit message 是指 git commit 时写的注释信息。如果你刚刚提交的注释（git commit -m "注释"）写的不清楚，在git push到远程仓库之前，**可以通过git commit --amend -m "新的注释"来替换刚才的注释**。如果你想在之前的注释上修改，则可以执行git commit --amend。

  **扩展**： 如果是你刚刚执行git commit，之后没有再次提交，也没有git push到远程仓库，在这种情况下，你还可以继续添加文件到这个commit中，只需要git add <新修改的文件>，然后git commit --amend修改注释，就可以在git log --name-only中看到新的注释和提交的新文件了。

- **在`git pull`之前，将本地的改动保存到储藏区（`git stash`）**。

- 在提交前，执行`git diff`查看改动。对于已经暂存的文件，采用`git diff --cached`或者`git diff --staged`来查看改动。这里的『暂存』是指`git add`之后的状态。

- 提交之后，用`git log`看一下提交历史。用`git log -p`查看更多的改动信息。用`git log --name-status`或`git log --name-only`查看修改了哪些文件。

- 抓取远程仓库的改动之前，先看看其他人都做了哪些改动。相关命令：

  1. `git fetch origin`获取远程origin仓库的改动到本地，但是还没有合并到本地工作目录中；
  2. `git log origin/master`查看origin仓库中master分支的改动记录。

- 发布本地分支时，保持服务器上分支名字与本地分支名字一样，前提是在创建本地分支时，起一个简洁明确的好名字。

  例如，新建的本地分支为 feature_xxx，发布这个分支：git push -u origin feature_xxx，其中-u是在本地分支 feature_xxx 和远程分支之间创建『跟踪』链接，对于同一个分支，-u只使用一次即可，不用每次都加-u。

- 删掉不用的分支。

  1. 本地分支 fix_bug_xxx 上修复了某个 bug，然后合并到了 master 分支上，就可以把 fix_bug_xxx 分支删掉了：`git branch -d fix_bug_xxx`。
  2. 本地分支 fix_bug_yyy 上有你的改动，你发布到远程仓库后，你的同事基于你的改动又做了新的改动，这些改动都是在 fix_bug_yyy 分支上（远程分支与本地分支同名，严格来说，是 origin/fix_bug_yyy，如果仓库名是 origin 的话）。你将 orgin/fix_bug_yyy 改动拉取到本地工作目录中，然后合并到 master 分支上。此后，你需要删除本地的 fix_bug_yyy 分支，还需要删除远程分支 orgin/fix_bug_yyy ：`git branch -dr orgin/fix_bug_yyy`。

- 及时恢复误删掉的 commit。如果`git reset`删除了已经提交的分支，可以通过`git reflog`看到被删掉的 commit。如果需要恢复commit，可以通过`git merge <deleted_commit>`来恢复。

- 设置自己顺手的 diff 工具和 merge 工具。参考:[设置 BeyondCompare 作为 diff 和 merge 工具](https://my.oschina.net/u/1010578/blog/348731)，[使用 meld 作为 diff 和 merge 工具](https://codeday.me/bug/20180109/116253.html)，[Use vimdiff as git mergetool](https://www.rosipov.com/blog/use-vimdiff-as-git-mergetool/)。
