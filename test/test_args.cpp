
#include <iostream>
#include <string>
#include <map>
#include <set>



class ArgsParser
{
public:
    /**
     * @brief 将每个选项解析出来，从第一个参数开始, 多参数模式 -a -L -file xxx -file xxx -more value1 value2
     * 
     * @param argc 
     * @param argv 
     * @return bool true -  解析成功
     */
    ArgsParser(int argc, const char *argv[])
    {
        std::string key;
        std::set<std::string> values;

        auto save_args = [&]{
                // 有新的参数产生， 保存之前的参数
                // key 为空，说明选项不存在，将值保存到unmatched_args_;                
                if (key.empty()) {
                    for (auto & v : values){                        
                        unmatched_args_.insert(v);
                    }
                } else {
                    // key 不为空，保存到 args 
                    // 如果key已存在，而且值不同，才需要插入
                    if (args_.count(key)){
                        auto & saved_values = args_[key];                        
                        for (auto & v : values){
                            saved_values.insert(v);
                        }
                    }else {
                        args_.emplace(std::make_pair(key, values));
                    }
                }
            };

        for (int i = 1; i < argc; i ++){
            std::string opt = argv[i];
            if (opt[0] == '-'){
                // 保存值
                save_args();
                // 重置
                key = opt;
                // 清空值
                decltype(values) tmp;
                tmp.swap(values);
            } else  {
                values.insert(opt);
            }
        }

        // 可能还有未保存的值
        save_args();
    }

    /// @brief 显示所有解析到的参数
    void dump()
    {
        auto value_to_string = [](std::set<std::string> const & values) -> std::string {
            std::string ret;
            for (auto & v : values){
                ret += "<" + v + "> ";
            } 
            if (ret.empty()){
                ret += "[bool]";
            }
            return ret;    
        };


        std::cout << "args:" << std::endl;
        for (auto & kv : args_)
        {
            std::cout << kv.first << " : " << value_to_string(kv.second) << std::endl;
        }

        std::cout << "unmatched args:" << std::endl;
        if (!unmatched_args_.empty())
        {
            std::cout << value_to_string(unmatched_args_) << std::endl;        
        }
    }

private:
    std::map<std::string, std::set<std::string>> args_;
    std::set<std::string> unmatched_args_;
};








int main(int argc, const char *argv[])
{
    // auto options = MultiOptioins("app_name");

    // options.add("-h,--help", "Print a helper");// bool 
    // options.add("-L", "Print a helper", opt::value<std::string>());
    // options.add("-a", "Print a helper", ); 
    // options.add("-a", "Print a helper");

    auto args = ArgsParser(argc, argv);

    args.dump();

    return 0;
}
