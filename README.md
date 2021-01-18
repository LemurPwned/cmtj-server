# CMTJ SERVER

The processing engine and the API for some functionalities from `cmtj` library.
Only basic services are implemented now.
The database used is [leveldb](https://github.com/google/leveldb).
The [Postman](https://www.postman.com/) collection is included here: [CMTJ collection](./CMTJ.postman_collection.json).

Please ubmit an issue if you wish some extra functionality added.
## Running the pre-built image
Some verion of the image is already prebuilt and available in the dockerhub under `lemurpwned/cmtj-server`.
So, if you want to run it, it's sufficient to:
```bash
docker run -p 8080:8080 lemurpwned/cmtj-server
```

See below in [Building the image section](#building-the-image) you wish to build your own image, especially if you modified the code.

## Retrieve the results
Most of the use cases will probably involve Python, so head to [example/task_runner.py](example/task_runner.py) 
to view an example of a code that can be run to fetch the results from the server.  
There are two tasks included:
* [VSD - Voltage spin diode](example/vsd_task.py)
* [PIM - Pulse Induced Magnetometry](example/pim_task.py)

Modify the code and observe the responses to check out different behaviours of the server.
## Building the image
Place all the `.hpp` files from the `cmtj` library in the folder named `third` (you need to create it in the root of the repository), 
so that the files may be copied into the container.  
Build the image, using the provided Dockerfile:
```bash 
docker build -t cmtj-server
```
Run the server:
```bash
docker run -p 8080:8080 cmtj-server
```
## Workflow

1. Submit a task to the `\queue` endpoint.
    ```bash
    curl -XPOST 'localhost:8080/queue' \
        --header 'Content-Type: application/json' \
        --data-raw '{....}'
    ```
    The response will look like so:
    ```json
    {
        "message": "Accepted onto queue",
        "uuid": "8b9a5da6-b9c1-4687-9b98-b516b3700bb3"
    }
    ```
    Use that `uuid` to query for the result of the task.
2. Query `\task` endpoint to fetch the result. 
    ```bash
    curl -XGET host:8080/task?uuid=XXXXXXXXXXX
    ```
See [Postman collection](./CMTJ.postman_collection.json). for examples of use.