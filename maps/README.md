# Site maps

One pair per site, `<site>.pgm` + `<site>.yaml`, the Nav2 map_server format, plus `<site>.scene.yaml` with
the site's named waypoints for natural-language commands ("go to the start", "go to stop 3").

**Where the map comes from.** The robot maps the site itself (`sudo drmap mapping -n <site>` then
`drmap stop_mapping` on the navigation host, 10.21.31.106, up to 50 x 50 m), and localizes on that map from
then on. Copy the result here so the laptop's map_server serves the same grid the robot's pose refers to:

```
scp user@10.21.31.106:/var/opt/robot/data/maps/<site>/occ_grid.pgm  maps/<site>.pgm
scp user@10.21.31.106:/var/opt/robot/data/maps/<site>/occ_grid.yaml maps/<site>.yaml
```

Then edit `image:` in `<site>.yaml` to `<site>.pgm`. Do not change `origin` or `resolution`: they define the
frame every vendor pose is expressed in (0.1 m cells; a 61 x 52 m map had origin [-37.8, -17.4]).

**Scene yaml** (`maps/<site>.scene.yaml`): named places in that map frame, x y yaw in metres and radians:

```yaml
waypoints:
  start: [0.0, 0.0, 0.0]
  dock:  [-3.2, 1.1, 3.14]
  stop1: [6.5, -2.0, 1.57]
```

Maps and scene files of real sites are committed; keyframes, object maps and recordings stay under `~/m20_sim`.
