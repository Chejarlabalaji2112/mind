class SkillAlreadyExistsError(Exception):
    """Exception raised when trying to create a skill that already exists."""
    pass


class SkillNotFoundError(Exception):
    """Exception raised when a skill is not found in the database."""
    pass    