
class RosgenError(Exception):
    """Base class for all ros2-codegen exceptions."""
    pass

class ConfigError(RosgenError):
    """Raised when the YAML file is missing or poorly formatted[cite: 151]."""
    pass

class ValidationError(RosgenError):
    """Raised when the YAML content fails schema rules[cite: 152]."""
    pass

class GenerationError(RosgenError):
    """Raised when file writing or template rendering fails[cite: 153]."""
    pass