workspace(name = "application-pnc")

#######################################APOLLO#######################################
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

new_local_repository(
    name = "apollo_src",
    build_file = "//tools/proto:apollo.BUILD",
    path = "/opt/apollo/neo/src",
)

http_archive(
    name = "remotejdk11_linux",
    sha256 = "360626cc19063bc411bfed2914301b908a8f77a7919aaea007a977fa8fb3cde1",
    urls = [
        "https://apollo-pkg-beta.bj.bcebos.com/bazel/zulu11.37.17-ca-jdk11.0.6-linux_x64.tar.gz",
    ],
)

http_archive(
    name = "go_sdk",
    sha256 = "0052d452af7742c8f3a4e0929763388a66403de363775db7e90adecb2ba4944b",
    urls = [
        "https://apollo-pkg-beta.bj.bcebos.com/bazel/go1.12.5.linux-amd64.tar.gz",
    ],
)

http_archive(
    name = "build_bazel_rules_apple",
    sha256 = "0052d452af7742c8f3a4e0929763388a66403de363775db7e90adecb2ba4944b",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/rules_apple.0.31.3.tar.gz",
        "https://github.com/bazelbuild/rules_apple/releases/download/0.31.3/rules_apple.0.31.3.tar.gz",
    ],
)

http_archive(
    name = "rules_foreign_cc",
    sha256 = "6041f1374ff32ba711564374ad8e007aef77f71561a7ce784123b9b4b88614fc",
    strip_prefix = "rules_foreign_cc-0.8.0",
    urls = [
        "https://apollo-system.bj.bcebos.com/archive/6.0/rules_foreign_cc-0.8.0.tar.gz",
        "https://github.com/bazelbuild/rules_foreign_cc/archive/0.8.0.tar.gz",
    ],
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

http_archive(
    name = "rules_cc",
    patches = ["//tools/package:rules_cc.patch"],
    sha256 = "4dccbfd22c0def164c8f47458bd50e0c7148f3d92002cdb459c2a96a68498241",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/rules_cc-0.0.1.tar.gz",
        "https://github.com/bazelbuild/rules_cc/releases/download/0.0.1/rules_cc-0.0.1.tar.gz",
    ],
)

load("//dev/bazel:deps.bzl", "init_deps")

init_deps()

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()
#######################################APOLLO#######################################
