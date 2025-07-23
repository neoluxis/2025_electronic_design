#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nlxpy.dl.model.model
========

Desc: Model factory and register for deep learning models.

Author: Neolux Lee

Date: 2025-06-13

Email: neolux_lee@outlook.com

Ver: 0.0.1
"""
import os
from typing import Callable
import logging


class Model:
    """
    Model registration and management class.
    """

    __module_registry: dict = {}

    def __init__(self, *args, **kwargs):
        pass

    @classmethod
    def register(cls, name: str, note: str | None = None) -> Callable:
        """Register the model with a given name."""

        def register_model(mcls):
            if name in cls.__module_registry:
                raise ValueError(f"Model '{name}' is already registered.")
            cls.__module_registry[name] = (mcls, note if note else mcls.__name__)
            logging.debug(f"Model '{name}' registered successfully.")
            return cls

        return register_model

    @classmethod
    def unregister(cls, name: str) -> bool:
        """Unregister the model with a given name."""
        if name in cls.__module_registry:
            del cls.__module_registry[name]
            logging.debug(f"Model '{name}' unregistered successfully.")
            return True
        else:
            logging.warning(f"Model '{name}' not found in registry.")
            return False

    @classmethod
    def get_registered_models(cls, proc=False) -> list | None:
        """
        Get a list of all registered models.

        :param proc: If True, return a list for following processing. If False, print it out
        """
        if proc:
            ret = []
            for name, (mcls, note) in cls.__module_registry.items():
                ret.append({"name": name, "desc": note})
            return ret
        else:
            if not cls.__module_registry:
                print("No models registered.")
            else:
                print("Registered models:")
                for idx, (name, (mcls, note)) in enumerate(
                    cls.__module_registry.items()
                ):
                    print(f"{idx + 1}. {name} - {note}")
            return None


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    # Example usage
    @Model.register("example_model")
    class ExampleModel(Model):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            print("ExampleModel initialized.")

    Model.get_registered_models(proc=False)

    # Access the registered model
    # model_instance = Model.__MODULE_REGISTRY["example_model"]()
    # print(f"Registered model: {model_instance.__class__.__name__}")

    # Unregister the model
    Model.unregister("example_model")
    print("Model 'example_model' unregistered.")

    Model.get_registered_models(proc=False)
