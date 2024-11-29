#pragma once

#include "glomap/controllers/global_mapper.h"
#include <iomanip>
#include <type_traits>

#include <functional>
#include <memory>
#include <optional>
#include <source_location>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace glomap {

    class OptionError : public std::runtime_error {
    public:
        explicit OptionError(const std::string& message,
                             const std::source_location& location = std::source_location::current())
            : std::runtime_error(CreateMessage(message, location)) {}

    private:
        static std::string CreateMessage(const std::string& message,
                                         const std::source_location& location) {
            std::ostringstream ss;
            ss << location.file_name() << ":" << location.line() << " - " << message;
            return ss.str();
        }
    };

    template <typename T>
    struct Option {
        T* value;
        std::string description;
        bool required;
        std::optional<T> default_value;

        Option() = delete;

        Option(T* v, std::string desc = "", bool req = false, std::optional<T> def = std::nullopt)
            : value(v),
              description(std::move(desc)),
              required(req),
              default_value(std::move(def)) {}
    };

    class OptionManager {
    public:
        explicit OptionManager(bool add_project_options = true);
        ~OptionManager() = default;

        // Delete copy and move operations
        OptionManager(const OptionManager&) = delete;
        OptionManager& operator=(const OptionManager&) = delete;
        OptionManager(OptionManager&&) = delete;
        OptionManager& operator=(OptionManager&&) = delete;

        void Parse(std::span<char*> args);
        void Reset();
        void PrintHelp() const;

        // Public storage
        std::string database_path;
        std::string image_path;
        std::unique_ptr<GlobalMapperOptions> mapper;

        // Option group methods
        void AddDatabaseOptions();
        void AddImageOptions();
        void AddMapperOptions();
        void AddMapperResumeOptions();
        void AddViewGraphCalibrationOptions();
        void AddRelativePoseEstimationOptions();
        void AddRotationEstimatorOptions();
        void AddTrackEstablishmentOptions();
        void AddGlobalPositionerOptions();
        void AddBundleAdjusterOptions();
        void AddTriangulatorOptions();
        void AddInlierThresholdOptions();

        template <typename T>
        void AddRequiredOption(const std::string& name, T* option,
                               const std::string& description) {
            AddOption<T>(name, option, description, true, std::optional<T>{});
        }

        // Option management methods
        template <typename T>
        void AddOption(const std::string& name, T* option,
                       const std::string& description = "",
                       bool required = false,
                       std::optional<T> default_value = std::optional<T>{}) {
            if (options_.find(name) != options_.end())
            {
                std::ostringstream ss;
                ss << "Option '" << name << "' already exists";
                throw OptionError(ss.str());
            }

            options_.emplace(name, Option<T>(option, description, required, default_value));
            if (default_value)
            {
                *option = *default_value;
            }
        }

    private:
        // Option value type definition
        using OptionValue = std::variant<
            Option<bool>,
            Option<int>,
            Option<double>,
            Option<std::string>>;

        struct OptionGroup {
            bool is_added = false;
            std::function<void()> add_options;
        };

        template <typename T>
        void AddOptionWithDefault(const std::string& name, T* option,
                                  const std::string& description,
                                  const T& default_value) {
            AddOption(name, option, description, false, std::optional<T>{default_value});
        }

        void InitializeDefaultOptions();
        void ValidateOptions() const;
        void ParseOption(const std::string& arg);

        // Storage
        std::unordered_map<std::string, OptionValue> options_;
        std::unordered_map<std::string, OptionGroup> option_groups_;
        bool initialized_ = false;
    };

} // namespace glomap