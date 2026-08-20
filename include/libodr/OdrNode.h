#pragma once

#include <cstddef>
#include <memory>

namespace odr
{

struct OdrNode
{
    OdrNode() = default;
    virtual ~OdrNode() = default;

    OdrNode(const OdrNode&) noexcept;
    OdrNode(OdrNode&&) noexcept;
    OdrNode& operator=(const OdrNode&) noexcept;
    OdrNode& operator=(OdrNode&&) noexcept;

    OdrNode*       parent() noexcept;
    const OdrNode* parent() const noexcept;

    void set_parent(OdrNode* parent);

    std::ptrdiff_t xml_offset = -1; // locate element in xml, is utf8-based offset for pugixml by default

private:
    std::shared_ptr<OdrNode*> self_;
    std::weak_ptr<OdrNode*>   parent_;
};

} // namespace odr
