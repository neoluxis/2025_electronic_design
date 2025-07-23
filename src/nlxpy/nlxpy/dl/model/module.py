#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nlxpy.dl.model.zoo
========

Desc: Model factory and register for deep learning models.
Author: Neolux Lee
Date: 2025-06-13
Email: neolux_lee@outlook.com
Ver: 0.0.1
"""

import os
import logging

import torch
import torch.nn as nn

from .model import Model as m


@m.register("MLP", note="Multi-Layer Perceptron")
class MLP(nn.Module):
    """
    Multi-Layer Perceptron (MLP) model.
    """

    def __init__(
        self,
        input_size: int = 784,
        out_classes: int = 10,
        hidden_sizes: list = [128, 64],
        dropout: float = 0.5,
    ):
        super(MLP, self).__init__()
        layers = []
        prev_size = input_size
        for size in hidden_sizes:
            layers.append(nn.Linear(prev_size, size))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout))
            prev_size = size
        layers.append(nn.Linear(prev_size, out_classes))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        """
        Forward pass of the MLP model.
        :param x: Input tensor.
        :return: Output tensor.
        """
        return self.model(x)


@m.register("Conv2dBlock", note="Convolutional Block 2D")
class ConvBlock(nn.Module):
    """
    Convolutional block with Conv2d, BatchNorm2d, and ReLU activation.
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        cfg=[[32, 3, 1, 1], [64, 3, 1, 1]],
        dropout: float = 0.5,
    ):
        super(ConvBlock, self).__init__()
        layers = []
        for c in cfg:
            layers.append(
                nn.Conv2d(
                    in_channels, c[0], kernel_size=c[1], stride=c[2], padding=c[3]
                )
            )
            layers.append(nn.BatchNorm2d(c[0]))
            layers.append(nn.ReLU())
            in_channels = c[0]
        layers.append(nn.Dropout(dropout))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        """
        Forward pass of the ConvBlock model.
        :param x: Input tensor.
        :return: Output tensor.
        """
        return self.model(x)

@m.register("Flatten", note="Flatten Layer")
class Flatten(nn.Module):
    """
    Flatten layer to convert multi-dimensional input to 1D.
    """

    def __init__(self, use_nn=False):
        super(Flatten, self).__init__()
        self.use_nn = use_nn
        if use_nn:
            self.flatten = nn.Flatten()
            

    def forward(self, x):
        """
        Forward pass of the Flatten layer.
        :param x: Input tensor.
        :return: Flattened tensor.
        """
        if self.use_nn:
            return self.flatten(x)
        else:
            return x.view(x.size(0), -1)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    m.get_registered_models()
