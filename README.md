[![CI](https://github.com/f3d-app/f3d/actions/workflows/ci.yml/badge.svg)](https://github.com/f3d-app/f3d/actions/workflows/ci.yml) [![codecov](https://codecov.io/gh/f3d-app/f3d/branch/master/graph/badge.svg?token=siwG82IXK7)](https://codecov.io/gh/f3d-app/f3d) [![Downloads](https://img.shields.io/github/downloads/f3d-app/f3d/total.svg)](https://github.com/f3d-app/f3d/releases) ![Discord](https://discordapp.com/api/guilds/1046005690809978911/widget.png?style=shield)

# F3D - Fast and minimalist 3D viewer
By Michael Migliore and Mathieu Westphal.

<img src="resources/logo.svg" align="left" width="20px"/>
F3D (pronounced `/fɛd/`) is a fast and minimalist 3D viewer. It supports many file formats, from digital content to scientific datasets (including glTF, STL, STEP, PLY, OBJ, FBX, Alembic), can show animations and support lot of rendering and texturing options including real time physically based rendering and raytracing.
<br clear="left"/>

It is fully controllable from the command line and support configuration files. It can provide thumbnails, support interactive hotkeys, drag&drop and integration into file managers.

F3D also contains the libf3d, a simple library to render meshes, with C++, Python and Java Bindings.

<img src="https://user-images.githubusercontent.com/3129530/194735416-3f386437-456c-4145-9b5e-6bb6451d7e9a.png" width="640">

*A typical render by F3D*

<img src="https://user-images.githubusercontent.com/3129530/194735261-dd6f1c1c-fa57-47b0-9d27-f735d18ccd5e.gif" width="640">

*Animation of a glTF file within F3D*

<img src="https://user-images.githubusercontent.com/3129530/194735272-5bcd3e7c-a333-41f5-8066-9b0bec9885e8.png" width="640">

*A direct scalars render by F3D*

See the [gallery](doc/GALLERY.md) for more images, take a look at the [changelog](doc/CHANGELOG.md) or go to the [releases page](https://github.com/f3d-app/f3d/releases) to download F3D!

If you need any help or want to discuss with other F3D users and developers, head over to our [discord](https://discord.gg/ZYHzNeEdX5).

# Quickstart

Open a file and visualize it interactively:

```
f3d /path/to/file.ext
```

Open a file and save the rendering into an image file:

```
f3d /path/to/file.ext --output=/path/to/img.png
```

Get help:

```
f3d --help
man f3d # Linux only
```

# Documentation

- To get started, please take a look at the [user documentation](doc/user/README_USER.md).
- If you need any help, are looking for a feature or found a bug, please open an [issue](https://github.com/f3d-app/f3d/issues).
- If you want to use the libf3d, please take a look at its [documentation](doc/libf3d/README_LIBF3D.md).
- If you want to build F3D and contribute to it, please take a look at the [developer documentation](doc/dev/README_DEV.md).

# Support

F3D is developed by a team of passionate devs. Please use F3D and star it on github to support us!

# Acknowledgments

F3D was initially created by [Kitware SAS](https://www.kitware.eu/) and is relying on many awesome open source projects, including [VTK](https://vtk.org/), [OCCT](https://dev.opencascade.org/), [Assimp](https://www.assimp.org/), [Alembic](http://www.alembic.io/) and [OSPRay](https://www.ospray.org/).

# License

F3D can be used and distributed under the 3-Clause BSD License, see the [license](LICENSE.md).
F3D rely on other libraries and tools, all under permissive licenses, see the [third party licenses](doc/THIRD_PARTY_LICENSES.md).
