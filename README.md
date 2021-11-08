# Replay Parser

This is the branch for handling parsing of replay files. 

## Prerequisites

Python >= 3.7 and Golang >= 1.16.
Pipenv (https://github.com/pypa/pipenv)

## Installation

`cd` to the directory of this repository and run:

`pipenv install`

#### Why?

It would be difficult to ask people who work on this branch to manually install the dependencies via `git clone` ("To install `csgo`, clone the repository by running `git clone https://github.com/pnxenopoulos/csgo`. Then, change directories to the newly cloned repository, and install the library by running `python setup.py install`."). It is also difficult to manage dependencies in a way that is easy to understand and maintain without using `pipenv`.

## Running Tests

Do not use the integrated terminal in VS Code as it activates the Python virtual environment.

Use `pipenv run mamba --format=documentation` in a standalone terminal to run the spec files in the spec folder

## Other Information

Please check the Wiki, which details a lot of important information (from branch `main`). Check it out:<br>
\>https://github.com/waldo-vision/video.analysis/wiki<
