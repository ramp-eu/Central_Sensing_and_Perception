# mod.sw.sp

OPIL module: Central Sensing & Perception (Central SP)
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

This is the Central SP. The docker image is created by executing in the root folder of this package:

```
docker build -t "centralsp:test" -f docker/centralSPdocker/Dockerfile .
```

Test built docker container by starting `docker-compose up` from the folder `test/docker_compose_files/Central_SP_docker. In docker-compose.yml uncomment the line with #, and comment the line above:
```
    sp:
        restart: always
        image: docker.ramp.eu/opil/opil.sw.sp.central:3.1.5
#        image: centralsp:test
```

Documentation about the Central SP module can be found here: <https://opil-documentation.readthedocs.io/en/latest/SP/Central_SP_Getting_Started.html>.
