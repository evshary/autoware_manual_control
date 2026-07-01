#include "core/parameter_reader.hpp"
#include "transport/zenoh/cli_params.hpp"

#include <yaml.h>

#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace autoware::manual_control
{

namespace
{

// Owns the parsed document so cleanup is a single point regardless of how the
// walk returns.
struct YamlDoc
{
  yaml_document_t doc{};
  bool ok = false;
  ~YamlDoc()
  {
    if (ok) {yaml_document_delete(&doc);}
  }
};

std::string scalar_of(yaml_node_t * n)
{
  return std::string(
    reinterpret_cast<const char *>(n->data.scalar.value),
    n->data.scalar.length);
}

// Recursively flattens the DOM into dotted names. A mapping recurses with the
// key appended to the prefix; a sequence stores its scalars as a vector; a
// scalar stores a one-element vector. libyaml resolves quotes, comments, flow
// style and UTF-8, so no string-edge logic lives here.
void walk(
  yaml_document_t * doc, yaml_node_t * n, const std::string & prefix,
  std::map<std::string, std::vector<std::string>> & out)
{
  if (!n) {return;}
  switch (n->type) {
    case YAML_MAPPING_NODE:
      for (auto * p = n->data.mapping.pairs.start; p < n->data.mapping.pairs.top; ++p) {
        yaml_node_t * k = yaml_document_get_node(doc, p->key);
        yaml_node_t * v = yaml_document_get_node(doc, p->value);
        if (!k || k->type != YAML_SCALAR_NODE) {continue;}
        const std::string key = scalar_of(k);
        walk(doc, v, prefix.empty() ? key : prefix + "." + key, out);
      }
      break;
    case YAML_SEQUENCE_NODE: {
        std::vector<std::string> items;
        for (auto * i = n->data.sequence.items.start; i < n->data.sequence.items.top; ++i) {
          yaml_node_t * e = yaml_document_get_node(doc, *i);
          if (e && e->type == YAML_SCALAR_NODE) {items.push_back(scalar_of(e));}
        }
        out[prefix] = std::move(items);
        break;
      }
    case YAML_SCALAR_NODE:
      out[prefix] = {scalar_of(n)};
      break;
    default:
      break;
  }
}

std::map<std::string, std::vector<std::string>> parse_config_file(const std::string & path)
{
  std::map<std::string, std::vector<std::string>> out;

  std::FILE * f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::fprintf(stderr, "[ParameterReader] cannot open config %s\n", path.c_str());
    return out;
  }

  yaml_parser_t parser;
  if (!yaml_parser_initialize(&parser)) {
    std::fprintf(stderr, "[ParameterReader] yaml parser init failed\n");
    std::fclose(f);
    return out;
  }
  yaml_parser_set_input_file(&parser, f);

  YamlDoc d;
  d.ok = yaml_parser_load(&parser, &d.doc);
  if (d.ok) {
    walk(&d.doc, yaml_document_get_root_node(&d.doc), "", out);
  } else {
    std::fprintf(
      stderr, "[ParameterReader] yaml parse error in %s: %s\n",
      path.c_str(), parser.problem ? parser.problem : "unknown");
  }

  yaml_parser_delete(&parser);
  std::fclose(f);
  return out;
}

// First --config <path> in argv.
std::string config_path_from_argv(int argc, char * argv[])
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") {return argv[i + 1];}}
  return "";
}

void warn(const std::string & name, const char * type, const std::string & v)
{
  std::fprintf(stderr, "[ParameterReader] %s: not a %s (%s)\n", name.c_str(), type, v.c_str());
}

} // namespace

namespace zenoh_params
{

const std::vector<std::string> * ParamMap::lookup(const std::string & name) const
{
  auto it = params_.find(name);
  return it != params_.end() ? &it->second : nullptr;
}

namespace
{
ParamMap g_loaded;
} // namespace

void parse(int argc, char * argv[])
{
  std::string cfg = config_path_from_argv(argc, argv);
  g_loaded = cfg.empty() ? ParamMap{} : ParamMap{parse_config_file(cfg)};
}

const ParamMap & loaded() {return g_loaded;}

} // namespace zenoh_params

struct ParameterReader::Impl
{
  const zenoh_params::ParamMap & params;
};

ParameterReader::ParameterReader(std::unique_ptr<Impl> impl)
: impl_(std::move(impl)) {}
ParameterReader::~ParameterReader() = default;

// Built here, where Impl is complete, so the gateway factory need not see it.
std::unique_ptr<ParameterReader> make_zenoh_parameter_reader()
{
  return std::make_unique<ParameterReader>(
    std::make_unique<ParameterReader::Impl>(ParameterReader::Impl{zenoh_params::loaded()}));
}

template<typename T>
T ParameterReader::read(const std::string & name, const T & default_val) const
{
  const std::vector<std::string> * v = impl_->params.lookup(name);
  if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    // A present key wins even when empty: `key: []` yields an empty vector (so an
    // empty `modes:` reaches the factory and aborts there). Only an absent key
    // falls back to the default.
    return v ? *v : default_val;
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    // Sequences are stored as strings; convert each element. Unlike the string
    // vector above, an empty `key: []` falls back to the default: a numeric
    // sequence (a pose preset) has no meaningful empty value, so an empty one reads
    // as unset. A non-numeric element also falls back (same contract as scalars).
    if (!v || v->empty()) {return default_val;}
    std::vector<double> out;
    out.reserve(v->size());
    for (const std::string & s : *v) {
      try {
        out.push_back(std::stod(s));
      } catch (const std::exception &) {
        warn(name, "number", s); return default_val;
      }
    }
    return out;
  } else {
    // An empty value is the "use the default" sentinel (preserves the original
    // empty-string contract).
    if (!v || v->empty() || v->front().empty()) {return default_val;}
    const std::string & s = v->front();
    if constexpr (std::is_floating_point_v<T>) {
      try {
        return static_cast<T>(std::stod(s));
      } catch (const std::exception &) {
        warn(name, "number", s); return default_val;
      }
    } else if constexpr (std::is_same_v<T, std::string>) {
      return s;
    } else {
      static_assert(sizeof(T) == 0, "unsupported parameter type");
    }
  }
}

template float ParameterReader::read<float>(const std::string &, const float &) const;
template double ParameterReader::read<double>(const std::string &, const double &) const;
template std::string ParameterReader::read<std::string>(
  const std::string &,
  const std::string &) const;
template std::vector<std::string>
ParameterReader::read<std::vector<std::string>>(
  const std::string &,
  const std::vector<std::string> &) const;
template std::vector<double>
ParameterReader::read<std::vector<double>>(
  const std::string &,
  const std::vector<double> &) const;

} // namespace autoware::manual_control
