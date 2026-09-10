#!/usr/bin/env python3

from ppts.models.enums import TransformProviderType

from transforms.tf_transform_provider import TFTransformProvider
from transforms.yaml_transform_provider import YAMLTransformProvider


class TransformProviderFactory:

    @staticmethod
    def create(
        config,
        node,
        tf_buffer=None,
    ):

        if config.provider == TransformProviderType.TF:

            return TFTransformProvider(
                tf_buffer=tf_buffer,
            )

        elif config.provider == TransformProviderType.YAML:

            return YAMLTransformProvider(
                yaml_file=config.yaml_file,
            )

        raise ValueError(
            f"Unsupported transform provider: "
            f"{config.provider}"
        )