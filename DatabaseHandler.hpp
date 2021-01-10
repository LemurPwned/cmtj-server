#ifndef DATABASE_HANDLER_H
#define DATABASE_HANDLER_H

#include <cassert>
#include <leveldb/db.h>
#include <leveldb/write_batch.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

using json = nlohmann::json;
class TaskDB
{
    leveldb::DB *db;
    leveldb::Options options;

public:
    TaskDB()
    {

        this->options.create_if_missing = true;
        leveldb::Status status = leveldb::DB::Open(options, "./testdb", &(this->db));
        if (!status.ok())
            spdlog::error("Status {}", status.ToString());
        assert(status.ok());
    }

    void saveFailedTaskResult(json input, std::exception &e)
    {
        spdlog::info("Saving failed task info...");
        std::string input_string = input.dump();
        std::string key = input["uuid"];

        json res;
        res["message"] = e.what();
        res["code"] = 304;

        leveldb::WriteBatch batch;
        batch.Put(key + "-input", input_string);
        batch.Put(key + "-output", res.dump());
        leveldb::Status s = this->db->Write(leveldb::WriteOptions(), &batch);
        if (!s.ok())
        {
            spdlog::error("Error {}", s.ToString());
        }
        else
        {
            spdlog::info("Status: {}", s.ToString());
        }
    }

    void saveTaskResult(json input, json output)
    {
        std::string input_string = input.dump();
        std::string output_string = output.dump();
        std::string key = input["uuid"];
        leveldb::WriteBatch batch;
        batch.Put(key + "-input", input_string);
        batch.Put(key + "-output", output_string);
        leveldb::Status s = this->db->Write(leveldb::WriteOptions(), &batch);

        if (!s.ok())
        {
            spdlog::error("Error {}", s.ToString());
        }
        else
        {
            spdlog::info("Status: {}", s.ToString());
        }
    }

    json retrieveTask(std::string uuid)
    {
        json response;
        std::string value;
        leveldb::Status s = this->db->Get(leveldb::ReadOptions(), uuid + "-output", &value);
        response["status"] = s.ToString();
        if (!s.ok())
        {
            response["code"] = 404;
            response["result"] = "Fetching failed";
        }
        else
        {
            response["code"] = 200;
            response["result"] = json::parse(value);
        }
        return response;
    }

    ~TaskDB()
    {
        delete this->db;
    }
};


#endif