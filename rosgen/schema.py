from pydantic import BaseModel, Field
from typing import List, Optional, Literal


# ----------------------------
# Interfaces
# ----------------------------

class InterfaceMessage(BaseModel):
    name: str
    fields: List[str]


class InterfaceService(BaseModel):
    name: str
    request: List[str]
    response: List[str]


class Interfaces(BaseModel):
    messages: List[InterfaceMessage] = []
    services: List[InterfaceService] = []


# ----------------------------
# Node I/O
# ----------------------------

class Publisher(BaseModel):
    topic: str
    msg_type: str
    qos: int = 10


class Subscriber(BaseModel):
    topic: str
    msg_type: str
    qos: int = 10
    callback: Optional[str] = None


class Node(BaseModel):
    name: str
    executable: str
    publishers: List[Publisher] = []
    subscribers: List[Subscriber] = []


# ----------------------------
# Package Metadata
# ----------------------------

class Maintainer(BaseModel):
    name: str
    email: Optional[str] = None


class PackageConfig(BaseModel):
    name: str
    version: str = "0.0.0"
    description: Optional[str] = None
    license: str = "Apache-2.0"
    maintainers: List[Maintainer] = []

    build_type: Literal["ament_python", "ament_cmake"] = "ament_python"

    build_dependencies: List[str] = []
    exec_dependencies: List[str] = []

    interfaces: Optional[Interfaces] = None
    nodes: List[Node] = []


# ----------------------------
# Top-Level Config
# ----------------------------

class Config(BaseModel):
    output_dir: str = "."
    packages: List[PackageConfig]

