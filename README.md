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
See Postman collection for examples of use.