# Git搭建本地仓库

## 本地

在本地电脑上，我将remote文件夹设置成git远程仓库，local文件夹可以pull/push到第一个文件夹

1. 在remote文件夹中初始化仓库： 

   打开终端，导航到您想要设置为远程仓库的文件夹，并执行以下命令

   ```bash
   cd /path/to/remote/repository
   git init --bare
   ```

   > 使用 `git init --bare` 命令时，它会创建一个裸仓库，裸仓库只包含版本控制所需的内容，没有工作目录。裸仓库通常用作远程仓库，用于共享和协作。

2. 在local文件夹中克隆远程仓库： 

   在您希望进行pull和push操作的第二个文件夹中，打开终端，并执行以下命令来克隆远程仓库：

   ```bash
   cd /path/to/local/repository
   git clone /path/to/remote/repository
   ```

3. 进行push和pull操作： 

   现在，您可以在local下的remote（clone下来的）中进行修改、提交和推送操作。

   在第local/remote中，使用终端进行代码修改和提交：

   ```bash
   cd /path/to/local/repository
   # 进行代码修改
   
   git add .
   git commit -m "提交的更改信息"
   ```

   然后，可以使用`git push`命令将更改推送到远程仓库：

   ```bash
   git push origin master
   ```

   若要从远程仓库获取最新更改，可以使用`git pull`命令：

   ```bash
   git pull origin master
   ```

在remote目录下

```bash
➜  remote git:(master) tree
.
├── branches
├── config
├── description
├── HEAD
├── hooks
│   ├── applypatch-msg.sample
│   ├── commit-msg.sample
│   ├── fsmonitor-watchman.sample
│   ├── post-update.sample
│   ├── pre-applypatch.sample
│   ├── pre-commit.sample
│   ├── pre-merge-commit.sample
│   ├── prepare-commit-msg.sample
│   ├── pre-push.sample
│   ├── pre-rebase.sample
│   ├── pre-receive.sample
│   └── update.sample
├── info
│   └── exclude
├── objects
│   ├── 3a
│   │   └── 13b822f4e7926bfc1e375eb1b9d1dbcc3233ef
│   ├── 5e
│   │   └── fb9bc29c482e023e40e0a2b3b7e49cec842034
│   ├── e6
│   │   └── 9de29bb2d1d6434b8b29ae775ad8c2e48c5391
│   ├── info
│   └── pack
└── refs
    ├── heads
    │   └── master
    └── tags

12 directories, 20 files
```

这些目录和文件是Git仓库的核心组成部分。以下是对每个目录和文件的简要说明：

- `branches`: 存储分支的相关信息，每个分支通常对应一个文件。
- `config`: 仓库的配置文件，包含与仓库相关的配置选项。
- `description`: 用于描述仓库的文件，一般包含仓库的简要描述信息。
- `HEAD`: 指向当前所在分支的符号链接。在默认情况下，它指向 `refs/heads/master`，即指向默认主分支。
- `hooks`: 存放Git钩子（hooks）的目录。Git钩子是可以在特定操作（如提交、推送等）发生时执行的自定义脚本。
- `info`: 包含仓库的附加信息，例如排除文件（exclude）和属性过滤器（attributes）。
- `objects`: 存储Git对象的目录。Git对象包括提交（commit）、树（tree）、标签（tag）和文件内容（blob）等。
- `refs`: 存储分支和标签的引用，每个分支和标签通常对应一个文件。

## 局域网

1. 配置远程仓库： 

   在担任远程仓库角色的计算机上，选择一个文件夹作为您的远程仓库目录，并执行以下命令：

   ```bash
   cd /path/to/remote/repository
   git init --bare
   ```

2. 获取远程仓库的IP地址或主机名： 

   在担任本地仓库角色的计算机上，您需要获取托管远程仓库的计算机的IP地址或主机名。这将用于设置本地仓库的远程地址。

3. 在本地仓库中添加远程地址： 

   在本地仓库所在的计算机上，导航到您的本地仓库文件夹，并执行以下命令：

   ```bash
   cd /path/to/local/repository
   git remote add origin <remote_address>
   ```

   其中，`<remote_address>` 是远程仓库的地址。您可以使用远程仓库的IP地址或主机名作为地址。

   例如，如果远程仓库的IP地址是 `192.168.0.100`，则命令应如下所示：

   ```bash
   git remote add origin 192.168.0.100:/path/to/remote/repository
   ```

## 如果已经链接了gitee上远程仓库

1. 查看已关联的远程仓库： 

   在本地仓库所在的文件夹中打开终端，并运行以下命令：

   ```bash
   cd /path/to/local/repository
   git remote -v
   ```

2. 添加额外的远程仓库（可选）： 

   如果您想要添加其他远程仓库，以便与多个远程仓库进行交互，可以使用`git remote add`命令。例如，假设您希望添加一个名为`second_remote`的远程仓库，可以执行以下命令：

   ```bash
   git remote add second_remote <second_remote_url>
   ```

   这将添加一个名为`second_remote`的远程仓库，并将其与指定的URL关联起来。

3. 修改远程仓库的URL（可选）： 

   如果需要修改已关联远程仓库的URL，可以使用`git remote set-url`命令。例如，假设您需要修改`origin`远程仓库的URL，可以执行以下命令：

   ```bash
   git remote set-url origin <new_remote_url>
   ```

   