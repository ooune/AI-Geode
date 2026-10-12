#pragma once

#include <condition_variable>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

// Serialises and writes memory files on a background thread so that saving a
// large Q-table never stalls the game. Jobs are keyed by path; a newer job for
// the same path replaces an older one that has not started yet.
class SaveWorker {
public:
    using Job = std::function<std::string()>; // produces the file contents

    SaveWorker();
    ~SaveWorker();
    SaveWorker(SaveWorker const&) = delete;
    SaveWorker& operator=(SaveWorker const&) = delete;

    void enqueue(std::filesystem::path path, Job job);
    // Blocks until all queued jobs are written (used before loading/importing).
    void flush();

private:
    void run();

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::condition_variable m_idle;
    std::map<std::filesystem::path, Job> m_jobs;
    bool m_busy = false;
    bool m_stop = false;
    std::thread m_thread;
};
