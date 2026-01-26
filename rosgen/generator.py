from pathlib import Path
from jinja2 import Environment, FileSystemLoader

from rosgen.schema import Config
from rosgen import normalizers
from rosgen.writers import write_text_file
from rosgen.utils import ensure_dir


TEMPLATE_DIR = Path(__file__).parent / "templates"

env = Environment(
    loader=FileSystemLoader(str(TEMPLATE_DIR)),
    autoescape=False,
    trim_blocks=True,
    lstrip_blocks=True,
)

print("TEMPLATES_DIR =", TEMPLATE_DIR)
print("Templates found:", list(TEMPLATE_DIR.rglob("*.j2")))
def generate_packages(cfg: Config) -> None:
    for pkg_cfg in cfg.packages:
        pkg = normalizers.normalize_package(pkg_cfg, cfg.output_dir)

        root = Path(cfg.output_dir) / pkg.package_name
        ensure_dir(root)

        _gen_package_xml(pkg, root)
        _gen_cmake(pkg, root)

        if pkg.build_type == "ament_python":
            _gen_python(pkg, root)

        if pkg.interfaces:
            _gen_interfaces(pkg, root)



def _gen_package_xml(pkg, root: Path) -> None:
    tmpl = env.get_template("package_xml.j2")
    write_text_file(root / "package.xml", tmpl.render(pkg=pkg))


def _gen_cmake(pkg, root: Path) -> None:
    tmpl = env.get_template("cmake/CMakeLists.txt.j2")
    write_text_file(root / "CMakeLists.txt", tmpl.render(pkg=pkg))


def _gen_python(pkg, root: Path) -> None:
    py_pkg_dir = root / pkg.package_name
    ensure_dir(py_pkg_dir)

    write_text_file(
        py_pkg_dir / "__init__.py",
        env.get_template("python/__init__.py.j2").render(pkg=pkg),
    )

    for node in pkg.nodes:
        write_text_file(
            py_pkg_dir / f"{node.executable}.py",
            env.get_template("python/node.py.j2").render(pkg=pkg, node=node),
        )

    write_text_file(
        root / "setup.py",
        env.get_template("python/setup.py.j2").render(pkg=pkg),
    )

    write_text_file(
        root / "setup.cfg",
        env.get_template("python/setup.cfg.j2").render(pkg=pkg),
    )


def _gen_interfaces(pkg, root: Path) -> None:
    for msg in pkg.interfaces.messages:
        ensure_dir(root / "msg")
        tmpl = env.get_template("interfaces/msg.j2")
        write_text_file(
            root / "msg" / f"{msg.name}.msg",
            tmpl.render(msg=msg),
        )

    for srv in pkg.interfaces.services:
        ensure_dir(root / "srv")
        tmpl = env.get_template("interfaces/srv.j2")
        write_text_file(
            root / "srv" / f"{srv.name}.srv",
            tmpl.render(srv=srv),
        )
