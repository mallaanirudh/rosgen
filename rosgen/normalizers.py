from dataclasses import dataclass
from typing import List, Optional

from rosgen.schema import PackageConfig, Maintainer
from rosgen.utils import snake_case


# ----------------------------
# Normalized Data Models
# ----------------------------

@dataclass
class NormalizedMsg:
    name: str
    fields: List[str]


@dataclass
class NormalizedSrv:
    name: str
    request: List[str]
    response: List[str]


@dataclass
class NormalizedInterfaces:
    messages: List[NormalizedMsg]
    services: List[NormalizedSrv]


@dataclass
class NormalizedPublisher:
    topic: str
    msg_type: str
    qos: int


@dataclass
class NormalizedSubscriber:
    topic: str
    msg_type: str
    qos: int
    callback: Optional[str]


@dataclass
class NormalizedNode:
    name: str
    executable: str
    publishers: List[NormalizedPublisher]
    subscribers: List[NormalizedSubscriber]


@dataclass
class NormalizedMaintainer:
    name: str
    email: Optional[str]


@dataclass
class NormalizedPackage:
    # Core
    package_name: str
    version: str
    description: Optional[str]
    license: str

    # New fields
    output_dir: str
    build_type: str
    build_dependencies: List[str]
    exec_dependencies: List[str]
    maintainers: List[NormalizedMaintainer]

    # ROS content
    interfaces: NormalizedInterfaces
    nodes: List[NormalizedNode]


# ----------------------------
# Public Normalizers
# ----------------------------

def normalize_package(pkg_cfg: PackageConfig, output_dir: str) -> NormalizedPackage:
    # Interfaces (may be None)
    if pkg_cfg.interfaces:
        messages = [
            NormalizedMsg(name=m.name, fields=m.fields)
            for m in pkg_cfg.interfaces.messages
        ]

        services = [
            NormalizedSrv(name=s.name, request=s.request, response=s.response)
            for s in pkg_cfg.interfaces.services
        ]
    else:
        messages = []
        services = []

    interfaces = NormalizedInterfaces(
        messages=messages,
        services=services,
    )

    # Nodes
    nodes: List[NormalizedNode] = []
    for n in pkg_cfg.nodes:
        pubs = [
            NormalizedPublisher(topic=p.topic, msg_type=p.msg_type, qos=p.qos)
            for p in n.publishers
        ]
        subs = [
            NormalizedSubscriber(
                topic=s.topic,
                msg_type=s.msg_type,
                qos=s.qos,
                callback=s.callback,
            )
            for s in n.subscribers
        ]

        nodes.append(
            NormalizedNode(
                name=n.name,
                executable=n.executable,
                publishers=pubs,
                subscribers=subs,
            )
        )

    # Maintainers
    maintainers = [
        NormalizedMaintainer(name=m.name, email=m.email)
        for m in pkg_cfg.maintainers
    ]

    return NormalizedPackage(
        package_name=snake_case(pkg_cfg.name),
        version=pkg_cfg.version,
        description=pkg_cfg.description,
        license=pkg_cfg.license,

        output_dir=output_dir,
        build_type=pkg_cfg.build_type,
        build_dependencies=pkg_cfg.build_dependencies,
        exec_dependencies=pkg_cfg.exec_dependencies,
        maintainers=maintainers,

        interfaces=interfaces,
        nodes=nodes,
    )
