#include "libodr/OdrNode.h"

namespace odr
{

OdrNode::OdrNode(const OdrNode&) noexcept : parent_(nullptr) {}

OdrNode::OdrNode(OdrNode&&) noexcept : parent_(nullptr) {}

OdrNode& OdrNode::operator=(const OdrNode&) noexcept
{
    parent_ = nullptr;
    return *this;
}

OdrNode& OdrNode::operator=(OdrNode&&) noexcept
{
    parent_ = nullptr;
    return *this;
}

OdrNode* OdrNode::parent() noexcept
{
    return parent_;
}

const OdrNode* OdrNode::parent() const noexcept
{
    return parent_;
}

void OdrNode::set_parent(OdrNode* parent)
{
    parent_ = parent;
}

} // namespace odr
