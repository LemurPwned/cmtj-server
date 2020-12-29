# CMTJ SERVER

The processing engine and the API for some functionalities from `cmtj` library.
Only basic services are implemented now.
The database used is `leveldb`.
## Building the image
Place all the `.hpp` files from the `cmtj` library in the folder named `third` (you need to create it in the root of the repository), so that the files may be copied into the container. 
Build the image, using the provided Dockerfile:
```bash 
docker build -t cmtj-server
```
Run the server:
```bash
docker run -p 8080:8080 cmtj-server
```
### Workflow

1. Submit a task to the `\queue` endpoint.
    ```bash
    curl -XPOST 'localhost:8080/queue' \
        --header 'Content-Type: application/json' \
        --data-raw '{....}'
    ```
2. Query `\task` endpoint to fetch the result. 
    ```bash
    curl -XGET host:8080/task?uuid=XXXXXXXXXXX
    ```
The task submission is of the following form:  
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
            "anis": [0, 0, 1],
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
