# Apollo Core Workspace

Apollob包管理开发调试样例工程，使用方法参考[包管理安装方式](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEBBCeIAcATnETFcgMxKZkgGxKAwkoDsa3YoAi45WdGSLxsYt0SzY%2BXICMa98oAMSgYALF7%2B2NhemsLBJsrCYZqKwors7AikBIp6miYmpFJSXpigFKhAA)

### 文件目录组织

```shell
.
├── core              # 依赖包配置，包括apollo核心包，以及一些工具包
├── profiles          # 整车应用配置，以单lidar、双camera车型为例
├── .env*             # 环境配置文件，包括容器名、镜像源等
├── .workspace.json*  # 软件包源配置文件，包括依赖的软件版本号
├── .buildtool*       # 编译配置文件
├── setup.sh          # 用于切换x86_64和aarch64架构配置
├── kill_all.sh       # 停止所有apollo相关进程
└── WORKSPACE         # bazel 的配置
```

## 安装步骤

## 更新AEM版本
如果您没有安装过Apollo环境管理工具，请参考 [安装 Apollo 环境管理工具](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEBBCeIAcATnETFcgMxKZkgGxKAwkoDsa3YoAi45WdGSLxsYt0SzY%2BXICMa98oAMSgYALF7%2B2NhemsLBJsrCYZqKwors7AikBIp6miYmpFJSXpigFKgAxAhEIMg1pHy8KUA) 进行安装。

如果您之前安装过AEM版本，10.0版本需要升级aem到最新版本，请现在宿主机环境执行:
```shell
sudo apt upate
sudo apt install apollo-neo-env-manager-dev --reinstall
```

### 初次安装
如果第一次安装，请在`git clone`后执行以下命令：
```shell
# 首先切换环境配置
bash setup.sh

# 启动容器
aem start

# 进入容器
aem enter

# 安装软件包
buildtool build

# 下载地图（此处下载sunnyvale，如果需要其他地图，可以用buildtool map list查看可供下载的地图）
buildtool map get sunnyvale

# 切换车辆配置 (您可以参考profiles目录下的sample编写自己的profile配置)
aem profile use sample

# 启动Dreamview+
aem bootstrap restart --plus
```
### 从旧版本升级
如果是从旧版升级（9.0.0-rc-r1或更老的版本），请在退出容器后`git pull`更新代码，然后执行以下命令：
```shell
# 首先切换环境配置
bash setup.sh

# 进入容器
aem enter

# 升级工具
buildtool upgrade

# 升级已经安装的软件包
buildtool build

# 下载地图（此处下载sunnyvale，如果需要其他地图，可以用buildtool map list查看可供下载的地图）
buildtool map get sunnyvale

# 切换车辆配置 (您可以参考profiles目录下的sample编写自己的profile配置)
aem profile use sample

# 启动Dreamview+
aem bootstrap restart --plus
```