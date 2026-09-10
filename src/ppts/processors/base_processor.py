from abc import ABC, abstractmethod


class BaseProcessor(ABC):
    """
    Base class for every PPTS processor.
    """

    @abstractmethod
    def process(self, context):
        """
        Process the PPTS context.

        Parameters
        ----------
        context : PPTSContext
            Shared pipeline context.

        Returns
        -------
        None
        """
        pass