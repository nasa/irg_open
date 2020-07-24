The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

irg_planetary_ephemeris
==================================

### Models
Models of the sun and some planets are in the `models` directory. They use
`IRGCelestialBodyPlugin` to allow them to be positioned in the sky. irg_sun
renders at a distance of 190000, while the planets render at a distance of
170000. These values more than different enough to prevent z-fighting when the
sun passes behind a planet, and they are close enough to be within the default
200000 far clip plane set by IRGNearFarClipPlugin.

There is currently no mechanism for setting celestial body render distances at
runtime. If you want to use different values you must make your own model.

