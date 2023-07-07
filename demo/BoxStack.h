/**
@file	DropBoxes.h

@author	Randy Gaul
@date	11/25/2014

        Copyright (c) 2014 Randy Gaul http://www.randygaul.net

        This software is provided 'as-is', without any express or implied
        warranty. In no event will the authors be held liable for any damages
        arising from the use of this software.

        Permission is granted to anyone to use this software for any purpose,
        including commercial applications, and to alter it and redistribute it
        freely, subject to the following restrictions:
                1. The origin of this software must not be misrepresented; you
must not claim that you wrote the original software. If you use this software in
a product, an acknowledgment in the product documentation would be appreciated
but is not required.
                2. Altered source versions must be plainly marked as such, and
must not be misrepresented as being the original software.
                3. This notice may not be removed or altered from any source
distribution.
*/

struct BoxStack : public Demo {
    virtual void Init() {
        // Create the floor
        q3Body* body = scene.CreateBody({});

        q3BoxDef boxDef;
        boxDef.m_restitution = 0;
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->SetBox(boxDef);

        boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));

        for (i32 i = 0; i < 8; ++i) {
            for (i32 j = 0; j < 8; ++j) {
                for (i32 k = 0; k < 10; ++k) {
                    body = scene.CreateBody({
                        .position = q3Vec3(-16.0f + 1.0f * j, 1.0f * i + 5.0f, -16.0f + 1.0f * k),
                        .bodyType = eDynamicBody,
                    });
                    body->SetBox(boxDef);
                }
            }
        }
    }

    virtual void Shutdown() { scene.RemoveAllBodies(); }
};
