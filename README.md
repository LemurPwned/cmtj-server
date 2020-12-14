# CMTJ SERVER

Contains the Engine from `cmtj` library to service demo requests.

## Building the image
Build the image first:
```bash 
docker build -t cmtj-server
```
Run the server:
```bash
docker run -p 8080:8080 cmtj-server
```


The requests are accepted by the `\queue` endpoint. The form of the request looks like this:
```json
{
    "vsdParams": {
        "amplitude": 10,
        "phase": 0,
        "frequencyStart": 7e9,
        "frequencyStep": 0.5e9,
        "fsteps": 10,
        "Hstart": [0,0,0],
        "Hstep": [0, 0, 1],
        "hsteps": 50,
        "time": 20e-9,
        "tStep": 1e-13,
        "tWrite": 1e-11,
        "tStart": 5e-9,
        "power": 10e-6
    },
    "drivers": [
        {
            "object": "anisotropy",
            "layer": "bottom",
            "subtype": "constant",
            "constantValue": 1500e3
        },
        {
            "object": "anisotropy",
            "layer": "free",
            "subtype": "constant",
            "constantValue": 650e3
        }
    ],
    "layers": [
        {
            "id": "bottom",
            "mag": [1,0,1],
            "anis": [0, 0, 1],,
            "Ms": 1000e3,
            "thickness": 1e-9,
            "cellSurface": 1e-7,
            "demagTensor": [...],
            "dipoleTensor": [...],
            "temperature": 0,
            "includeSTT": false,
            "damping": 0.011,
            "currentDensity": 0,
            "SlonczewskiSpacerLayerParameter": 1,
            "beta": 0,
            "spinPolarisation": 0
        },
        {
            "id": "free",
            "mag": [1,0,1],
            "anis": [0, 0, 1],
            "Ms": 1200e3,
            "thickness": 1e-9,
            "cellSurface": 1e-7,
            "demagTensor": [...],
            "dipoleTensor": [...],
            "temperature": 0,
            "includeSTT": false,
            "damping": 0.011,
            "currentDensity": 0,
            "SlonczewskiSpacerLayerParameter": 1,
            "beta": 0,
            "spinPolarisation": 0
        }
    ]
}
```
