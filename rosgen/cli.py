import click
from rosgen.config import load_config
from rosgen.schema import Config
from rosgen.generator import generate_packages  
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
    generate_packages(cfg)  # orchestrates template rendering + writes files
    click.echo("ROS 2 package generated successfully")
def main():
    cli()
