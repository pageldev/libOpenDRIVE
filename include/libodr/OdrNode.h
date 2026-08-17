#pragma once

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

private:
    std::shared_ptr<OdrNode*> self_;
    std::weak_ptr<OdrNode*>   parent_;
};

} // namespace odr
