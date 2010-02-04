
/* Copyright (c) 2009-2010, Stefan Eilemann <eile@equalizergraphics.com> 
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *  
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef EQSERVER_LAYOUT_H
#define EQSERVER_LAYOUT_H

#include "visitorResult.h" // enum
#include "types.h"

#include <eq/client/layout.h> // base class
#include <string>

namespace eq
{
namespace server
{
    class LayoutVisitor;
    struct LayoutPath;
    struct ViewPath;

    /**
     * The layout. @sa eq::Layout
     */
    class Layout : public eq::Object
    {
    public:
        /** 
         * Constructs a new Layout.
         */
        EQSERVER_EXPORT Layout();

        /** Creates a new, deep copy of a layout. */
        Layout( const Layout& from, Config* config );

        /** Destruct this layout. */
        virtual ~Layout();

        /**
         * @name Data Access
         */
        //@{
        /** Add a new view to this layout. */
        EQSERVER_EXPORT void addView( View* view );
        
        /** remove the view from this layout. */
        bool removeView( View* view );
        
        /** Get the list of views. */
        const ViewVector& getViews() const { return _views; }

        /** 
         * Find the first view of a given name.
         * 
         * @param name the name of the view to find
         * @return the first view with the name, or <code>0</code> if no
         *         view with the name exists.
         */
        View* findView( const std::string& name );

        Config* getConfig() { return _config; }
        const Config* getConfig() const { return _config; }

        /** @return the view of the given path. */
        View* getView( const ViewPath& path );

        /** @return the index path to this layout. */
        LayoutPath getPath() const;
        //@}

        /**
         * @name Operations
         */
        //@{
        /** 
         * Traverse this layout and all children using a layout visitor.
         * 
         * @param visitor the visitor.
         * @return the result of the visitor traversal.
         */
        VisitorResult accept( LayoutVisitor& visitor );
        VisitorResult accept( LayoutVisitor& visitor ) const;

        /** Unmap this layout and all its children. */
        void unmap();
        //@}
        
    protected:
        /** @sa Object::serialize */
        virtual void serialize( net::DataOStream& os, 
                                          const uint64_t dirtyBits );

    private:
        virtual void getInstanceData( net::DataOStream& os );

        /** The parent Config. */
        Config* _config;
        friend class Config;

        /** Child views on this layout. */
        ViewVector _views;

        union // placeholder for binary-compatible changes
        {
            char dummy[64];
        };
    };

    std::ostream& operator << ( std::ostream& os, const Layout* layout);
}
}
#endif // EQSERVER_LAYOUT_H
