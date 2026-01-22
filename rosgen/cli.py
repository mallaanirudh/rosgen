import click
@click.group()
def cli():
    """A handy CLI tool."""
    pass
@cli.command()
def hello():
    click.echo("hello")
if __name__ == '__main__':
    cli()
