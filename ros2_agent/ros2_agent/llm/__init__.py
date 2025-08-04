"""LLM module for Multi-Robot System."""

from .model import initialize_llm
from .specialist_models import call_vision_specialist

__all__ = ['initialize_llm', 'call_vision_specialist']