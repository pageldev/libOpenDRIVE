#include "libodr/OdrNode.h"

#include <utility>

namespace odr
{

OdrNode::OdrNode(const OdrNode& other) noexcept : xml_offset(other.xml_offset) {}

OdrNode::OdrNode(OdrNode&& other) noexcept : xml_offset(other.xml_offset), self_(std::move(other.self_)), parent_(std::move(other.parent_))
{
    if (self_)
        *self_ = this;
    other.parent_.reset();
}

OdrNode& OdrNode::operator=(const OdrNode& other) noexcept
{
    xml_offset = other.xml_offset;
    parent_.reset();
    return *this;
}

OdrNode& OdrNode::operator=(OdrNode&& other) noexcept
{
    if (this == &other)
        return *this;

    if (other.self_)
    {
        self_ = std::move(other.self_);
        *self_ = this;
    }
    xml_offset = other.xml_offset;
    parent_ = std::move(other.parent_);
    other.parent_.reset();
    return *this;
}

OdrNode* OdrNode::parent() noexcept
{
    const std::shared_ptr<OdrNode*> parent = parent_.lock();
    return parent ? *parent : nullptr;
}

const OdrNode* OdrNode::parent() const noexcept
{
    const std::shared_ptr<OdrNode*> parent = parent_.lock();
    return parent ? *parent : nullptr;
}

void OdrNode::set_parent(OdrNode* parent)
{
    if (parent && !parent->self_)
        parent->self_ = std::make_shared<OdrNode*>(parent);
    parent_ = parent ? parent->self_ : std::shared_ptr<OdrNode*>{};
}

} // namespace odr
