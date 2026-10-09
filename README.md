[![GitHub issues](https://img.shields.io/github/issues/davewalker5/OctopusController)](https://github.com/davewalker5/OctopusController/issues)
[![Releases](https://img.shields.io/github/v/release/davewalker5/OctopusController.svg?include_prereleases)](https://github.com/davewalker5/OctopusController/releases)
[![License](https://img.shields.io/badge/License-mit-blue.svg)](https://github.com/davewalker5/OctopusController/blob/main/LICENSE)
[![Language](https://img.shields.io/badge/language-python-blue.svg)](https://www.python.org)
[![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/davewalker5/OctopusController)](https://github.com/davewalker5/OctopusController/)

# Octopus Controller

<p>
<img src="https://github.com/davewalker5/OctopusController/blob/main/images/behaviour-showcase.gif" alt="Behaviour Showcase" width="600">
</p>

*A simulation of octopus-inspired distributed arm control, local sensing, obstacle avoidance and grasping.*

## Introduction

An octopus presents a fascinating problem in biological control. Unlike animals whose limbs are controlled predominantly through a central nervous system, an octopus has a highly distributed nervous system, with a substantial proportion of its neurons located in its eight arms.

Each arm is capable of complex, flexible movement and possesses extensive sensory capabilities. Rather than requiring the brain to specify every detail of an arm's movement, much of the processing involved in controlling and responding to the environment can take place locally.

This raises an interesting computational question: **how much complex, apparently intelligent behaviour can emerge from relatively simple, locally controlled mechanisms?**

Octopus Controller is an experimental simulation inspired by this question. It explores how multiple flexible arms can operate through a combination of movement control, local sensory feedback and responses to environmental constraints.

The objective is not to reproduce the octopus nervous system in complete biological detail, but to investigate some of the principles underlying distributed control and embodied behaviour.

### What the simulation demonstrates

The project currently explores five progressively more sophisticated capabilities:

1. **Single-arm control** — controlling the movement and curvature of an individual flexible arm towards a target
2. **Multi-arm control** — extending the control system to eight arms capable of operating simultaneously
3. **Local sensing** — allowing individual arms to respond to objects detected in their immediate environment
4. **Obstacle avoidance** — modifying arm movements in response to obstacles encountered while reaching towards a target
5. **Grasping** — demonstrating object interaction through the coordinated positioning and curling of an arm

Together, these demonstrations illustrate a progression from simple movement control towards more complex, environmentally responsive behaviour.

### Project scope

Octopus Controller is a computational experiment in biologically inspired control, rather than a biologically validated simulation of octopus anatomy or neurophysiology.

Its primary purpose is to explore how local control, sensory feedback and the coordination of multiple flexible appendages can produce useful and potentially complex behaviours without requiring every movement to be explicitly choreographed by a central controller.

The project provides a foundation for further experiments involving tactile feedback, arm cooperation, autonomous exploration and increasingly sophisticated interactions with the environment.

## Getting Started

Please see the [Wiki](https://github.com/davewalker5/OctopusController/wiki) for the user guide, controls and further details.

## Authors

- **Dave Walker** - _Initial work_

## Feedback

To report an issue or suggest an improvement, please use the project's [GitHub Issues](https://github.com/davewalker5/OctopusController/issues) page.

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
