import click
from rosgen.config import load_config
from rosgen.schema import Config
from rosgen.generator import generate_package  # assuming this function exists

@click.group()
def cli():
    """ROS 2 code generator."""
    pass

@cli.command()
@click.option('--config', default='ros2_codegen.yaml', help='Path to YAML config file')
def validate(config):
    """Validate ros2_codegen.yaml."""
    raw = load_config(config)
    Config(**raw)
    click.echo("Config is valid")

@cli.command()
@click.option('--config', default='ros2_codegen.yaml', help='Path to YAML config file')
def generate(config):
    """Generate ROS 2 package from YAML."""
    raw = load_config(config)
    cfg = Config(**raw)  # validate + normalize
    generate_package(cfg)  # orchestrates template rendering + writes files
    click.echo("ROS 2 package generated successfully")
import click

from rosgen.config import load_config
from rosgen.schema import Config
from rosgen.generator import generate_package


@click.group()
def cli():
    """ROS 2 code generator."""
    pass


@cli.command()
@click.option('--config', default='ros2_codegen.yaml')
def validate(config):
    raw = load_config(config)
    Config(**raw)
    click.echo("Config is valid")


@cli.command()
@click.option('--config', default='ros2_codegen.yaml')
def generate(config):
    raw = load_config(config)
    cfg = Config(**raw)

    generate_package(cfg)

    click.echo("Generation complete")


def main():
    cli()
