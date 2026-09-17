#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Generic, TypeVar


InputType = TypeVar("InputType")
OutputType = TypeVar("OutputType")


class BaseDetectorAlgorithm(
    ABC,
    Generic[InputType, OutputType],
):
    """
    Base interface for perception detectors.
    """

    @abstractmethod
    def detect(
        self,
        data: InputType,
    ) -> OutputType:
        """
        Detect objects from processed perception data.
        """
        raise NotImplementedError

class BasePathAlgorithm(
    ABC,
    Generic[InputType, OutputType],
):

    @abstractmethod
    def generate(
        self,
        data: InputType,
    ) -> OutputType:
        """
        Generate a path from the input perception result.
        """
        raise NotImplementedError