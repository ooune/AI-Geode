#include "SaveWorker.hpp"

#include <aigd/Storage.hpp>

#include <Geode/loader/Log.hpp>

SaveWorker::SaveWorker() : m_thread([this] { run(); }) {}

SaveWorker::~SaveWorker() {
    {
        std::lock_guard lock(m_mutex);
        m_stop = true;
    }
    m_cv.notify_all();
    // At process exit Windows may already have terminated this thread; never
    // block shutdown on it. Files are written atomically, so an interrupted
    // write leaves the previous version intact.
    if (m_thread.joinable()) m_thread.detach();
}

void SaveWorker::enqueue(std::filesystem::path path, Job job) {
    {
        std::lock_guard lock(m_mutex);
        m_jobs[std::move(path)] = std::move(job);
    }
    m_cv.notify_one();
}

void SaveWorker::flush() {
    std::unique_lock lock(m_mutex);
    m_idle.wait(lock, [this] { return m_jobs.empty() && !m_busy; });
}

void SaveWorker::run() {
    while (true) {
        std::filesystem::path path;
        Job job;
        {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [this] { return m_stop || !m_jobs.empty(); });
            if (m_stop && m_jobs.empty()) return;
            auto it = m_jobs.begin();
            path = it->first;
            job = std::move(it->second);
            m_jobs.erase(it);
            m_busy = true;
        }
        std::string error;
        std::string data = job();
        if (!aigd::storage::writeFileAtomic(path, data, &error)) {
            geode::log::error("Saving {} failed: {}", path.string(), error);
        }
        {
            std::lock_guard lock(m_mutex);
            m_busy = false;
        }
        m_idle.notify_all();
    }
}
