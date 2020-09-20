//  Source code dump of Box2D scene: rubegoldberg.rube
//
//  Created by R.U.B.E 1.7.4
//  Using Box2D version 2.3.0
//  Sun June 21 2020 21:54:58
//
//  This code is originally intended for use in the Box2D testbed,
//  but you can easily use it in other applications by providing
//  a b2World for use as the 'm_world' variable in the code below.

using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.Joints.Prismatic;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.WorldTests
{
    [ExcludeFromCodeCoverage]
    public static class RubeGoldbergWorld
    {
        public static World CreateWorld(out Body[] bodies, out Joint[] joints)
        {
            var gravity = new Vector2(0.000000000000000e+00f, -1.000000000000000e+01f);
            var world = new World(gravity);

            bodies = new Body[51];
            joints = new Joint[21];

            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(5.971687316894531e+00f, -8.106019020080566e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                bd.enabled = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[0] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.426122665405273e+00f, -6.587452411651611e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[1] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.067706227302551e-02f, -3.479100167751312e-01f);
                    vs[1] = new Vector2(8.067706227302551e-02f, 3.482054471969604e-01f);
                    vs[2] = new Vector2(-7.874175906181335e-02f, 3.482054471969604e-01f);
                    vs[3] = new Vector2(-7.874175906181335e-02f, -3.479098975658417e-01f);
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[1].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(6.150398731231689e+00f, -6.973721504211426e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[2] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.152114868164062e-01f, -3.786473572254181e-01f);
                    vs[1] = new Vector2(1.152114868164062e-01f, 3.899831771850586e-01f);
                    vs[2] = new Vector2(-1.131136417388916e-01f, 3.899831771850586e-01f);
                    vs[3] = new Vector2(-1.131136417388916e-01f, -3.786471188068390e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[2].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.491883754730225e+00f, -5.986517429351807e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[3] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.482706218957901e-01f, 3.280214071273804e-01f);
                    vs[1] = new Vector2(-8.590709418058395e-03f, 3.564623296260834e-01f);
                    vs[2] = new Vector2(-1.327803283929825e-01f, -3.284855782985687e-01f);
                    vs[3] = new Vector2(2.408098801970482e-02f, -3.569265007972717e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[3].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.960605621337891e+00f, -6.547102928161621e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[4] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(3.141191303730011e-01f, -1.742075979709625e-01f);
                    vs[1] = new Vector2(3.141191303730011e-01f, 1.602589339017868e-01f);
                    vs[2] = new Vector2(-3.225626945495605e-01f, 1.602589339017868e-01f);
                    vs[3] = new Vector2(-3.225626945495605e-01f, -1.742075234651566e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[4].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.951690673828125e+00f, -6.028742790222168e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[5] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(3.248913884162903e-01f, -1.461551934480667e-01f);
                    vs[1] = new Vector2(3.248913884162903e-01f, 1.543006747961044e-01f);
                    vs[2] = new Vector2(-3.124761283397675e-01f, 1.543006747961044e-01f);
                    vs[3] = new Vector2(-3.124761283397675e-01f, -1.461551636457443e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[5].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.928072929382324e+00f, -5.287778377532959e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[6] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.842054724693298e-01f, -1.876007169485092e-01f);
                    vs[1] = new Vector2(1.842054724693298e-01f, 1.883789598941803e-01f);
                    vs[2] = new Vector2(-1.748978495597839e-01f, 1.883789598941803e-01f);
                    vs[3] = new Vector2(-1.748978495597839e-01f, -1.876006871461868e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[6].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.825547695159912e+00f, -7.672395706176758e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[7] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.480759680271149e-01f, -3.798889219760895e-01f);
                    vs[1] = new Vector2(1.480759680271149e-01f, 3.887415528297424e-01f);
                    vs[2] = new Vector2(-8.024924993515015e-02f, 3.887415528297424e-01f);
                    vs[3] = new Vector2(-8.024924993515015e-02f, -3.798889219760895e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[7].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.955581188201904e+00f, -6.287708759307861e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[8] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.744700312614441e-01f, -1.249117255210876e-01f);
                    vs[1] = new Vector2(2.744700312614441e-01f, 1.298676133155823e-01f);
                    vs[2] = new Vector2(-2.660023868083954e-01f, 1.298676133155823e-01f);
                    vs[3] = new Vector2(-2.660023868083954e-01f, -1.249116957187653e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[8].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(6.395890712738037e+00f, -5.987843513488770e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[9] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.487545818090439e-01f, -3.301024138927460e-01f);
                    vs[1] = new Vector2(1.272160839289427e-02f, 3.525920212268829e-01f);
                    vs[2] = new Vector2(-1.436236351728439e-01f, 3.214388489723206e-01f);
                    vs[3] = new Vector2(-7.590693421661854e-03f, -3.612554669380188e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[9].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.947129249572754e+00f, -5.753320217132568e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[10] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(4.110610783100128e-01f, -1.919811218976974e-01f);
                    vs[1] = new Vector2(4.110610783100128e-01f, 1.856334656476974e-01f);
                    vs[2] = new Vector2(-3.899859189987183e-01f, 1.856334656476974e-01f);
                    vs[3] = new Vector2(-3.899859189987183e-01f, -1.919810473918915e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[10].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.924207210540771e+00f, -5.488777160644531e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[11] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(7.729114592075348e-02f, -8.702836930751801e-02f);
                    vs[1] = new Vector2(7.729114592075348e-02f, 1.265033483505249e-01f);
                    vs[2] = new Vector2(-8.333943784236908e-02f, 1.265033483505249e-01f);
                    vs[3] = new Vector2(-8.333943784236908e-02f, -8.702835440635681e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[11].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.795244693756104e+00f, -6.978021621704102e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[12] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.173663735389709e-01f, -3.785233795642853e-01f);
                    vs[1] = new Vector2(1.173663735389709e-01f, 3.901071250438690e-01f);
                    vs[2] = new Vector2(-1.109587550163269e-01f, 3.901071250438690e-01f);
                    vs[3] = new Vector2(-1.109587550163269e-01f, -3.785232603549957e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[12].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(6.166951179504395e+00f, -7.688251495361328e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[13] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.290136575698853e-01f, -3.886288404464722e-01f);
                    vs[1] = new Vector2(1.290136575698853e-01f, 3.800015449523926e-01f);
                    vs[2] = new Vector2(-9.931153059005737e-02f, 3.800015449523926e-01f);
                    vs[3] = new Vector2(-9.931153059005737e-02f, -3.886288404464722e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[13].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(6.447659015655518e+00f, -6.578778743743896e+00f);
                bd.angle = -3.138845443725586e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[14] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 3.000000119209290e-01f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 9.999999776482582e-03f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(7.564508914947510e-02f, -3.520573079586029e-01f);
                    vs[1] = new Vector2(7.564508914947510e-02f, 3.440580368041992e-01f);
                    vs[2] = new Vector2(-8.377373218536377e-02f, 3.440580368041992e-01f);
                    vs[3] = new Vector2(-8.377373218536377e-02f, -3.520572483539581e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[14].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-9.962106704711914e+00f, -5.336883068084717e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[15] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(5.082712173461914e-01f, -1.341846942901611e+00f);
                    vs[1] = new Vector2(1.662740707397461e-01f, 1.942352294921875e+00f);
                    vs[2] = new Vector2(-2.583751678466797e-01f, 1.925170898437500e+00f);
                    vs[3] = new Vector2(-4.917230606079102e-01f, -1.338597297668457e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[15].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(7.153423309326172e+00f, 1.670340776443481e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[16] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 2.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(5.563859939575195e-01f, -1.749677062034607e-01f);
                    vs[1] = new Vector2(5.563859939575195e-01f, 1.749676913022995e-01f);
                    vs[2] = new Vector2(3.803986907005310e-01f, 1.749676913022995e-01f);
                    vs[3] = new Vector2(3.803986907005310e-01f, -1.749676764011383e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[16].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(3.814836144447327e-01f, 2.922968566417694e-02f);
                    vs[1] = new Vector2(-3.814836442470551e-01f, 2.922968566417694e-02f);
                    vs[2] = new Vector2(-3.814836442470551e-01f, -2.922968380153179e-02f);
                    vs[3] = new Vector2(3.814835846424103e-01f, -2.922968938946724e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[16].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 2.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-3.803986907005310e-01f, -1.749677509069443e-01f);
                    vs[1] = new Vector2(-3.803986907005310e-01f, 1.749676167964935e-01f);
                    vs[2] = new Vector2(-5.563859939575195e-01f, 1.749676167964935e-01f);
                    vs[3] = new Vector2(-5.563859939575195e-01f, -1.749677807092667e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[16].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.183950233459473e+01f, 3.602335691452026e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[17] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[17].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[18] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.622321891784668e+01f, 4.735573291778564e+00f);
                    vs[1] = new Vector2(1.601119422912598e+01f, 4.735989570617676e+00f);
                    vs[2] = new Vector2(1.602764320373535e+01f, 3.814474105834961e+00f);
                    vs[3] = new Vector2(1.621816253662109e+01f, 3.825283050537109e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.095816612243652e+01f, 5.434564113616943e+00f);
                    vs[1] = new Vector2(8.198617935180664e+00f, 5.453726768493652e+00f);
                    vs[2] = new Vector2(8.219645500183105e+00f, 5.263209342956543e+00f);
                    vs[3] = new Vector2(1.076708602905273e+01f, 5.244045734405518e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(7.488466739654541e+00f, 1.697601437568665e+00f);
                    vs[1] = new Vector2(7.045552253723145e+00f, 1.697015643119812e+00f);
                    vs[2] = new Vector2(7.131743907928467e+00f, 1.522507667541504e+00f);
                    vs[3] = new Vector2(7.414176464080811e+00f, 1.515602588653564e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = true;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.012120485305786e+00f, -1.203510761260986e+00f);
                    vs[1] = new Vector2(8.127925872802734e+00f, -1.203510761260986e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-2.384965658187866e+00f, 1.565266609191895e+01f);
                    vs[1] = new Vector2(-2.384965658187866e+00f, 1.589544010162354e+01f);
                    vs[2] = new Vector2(-7.031423568725586e+00f, 1.606185531616211e+01f);
                    vs[3] = new Vector2(-7.031423568725586e+00f, 1.581908035278320e+01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.623795509338379e+00f, 7.986671447753906e+00f);
                    vs[1] = new Vector2(8.623795509338379e+00f, 8.229445457458496e+00f);
                    vs[2] = new Vector2(7.950877189636230e+00f, 8.229445457458496e+00f);
                    vs[3] = new Vector2(7.950877189636230e+00f, 7.986671447753906e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-5.547423362731934e-01f, 1.375105190277100e+01f);
                    vs[1] = new Vector2(-7.455649375915527e-01f, 1.375192642211914e+01f);
                    vs[2] = new Vector2(-7.635607719421387e-01f, 7.908406257629395e+00f);
                    vs[3] = new Vector2(-5.727376937866211e-01f, 7.907532691955566e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(6.170930862426758e-01f, 3.991966247558594e+00f);
                    vs[1] = new Vector2(5.832719802856445e-01f, 4.588045120239258e+00f);
                    vs[2] = new Vector2(-4.010983705520630e-01f, 4.584075927734375e+00f);
                    vs[3] = new Vector2(-3.672742247581482e-01f, 3.987997055053711e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-5.345141887664795e-01f, 1.373417568206787e+01f);
                    vs[1] = new Vector2(-2.392356634140015e+00f, 1.590833759307861e+01f);
                    vs[2] = new Vector2(-2.537431478500366e+00f, 1.578436946868896e+01f);
                    vs[3] = new Vector2(-6.795890331268311e-01f, 1.361020755767822e+01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.038518905639648e+01f, 1.389041066169739e+00f);
                    vs[1] = new Vector2(2.038518905639648e+01f, 1.557285785675049e+00f);
                    vs[2] = new Vector2(1.947545051574707e+01f, 1.557285785675049e+00f);
                    vs[3] = new Vector2(1.947545051574707e+01f, 1.389041066169739e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(7.342308521270752e+00f, 2.300606012344360e+00f);
                    vs[1] = new Vector2(7.298935890197754e+00f, 1.148176479339600e+01f);
                    vs[2] = new Vector2(7.056161880493164e+00f, 1.149811553955078e+01f);
                    vs[3] = new Vector2(7.056161880493164e+00f, 2.300606012344360e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(4.683557033538818e+00f, 5.022264480590820e+00f);
                    vs[1] = new Vector2(4.672747135162354e+00f, 5.212782859802246e+00f);
                    vs[2] = new Vector2(4.358123302459717e+00f, 5.211514472961426e+00f);
                    vs[3] = new Vector2(4.368934154510498e+00f, 5.020996093750000e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.619143486022949e+00f, 1.579549884796143e+01f);
                    vs[1] = new Vector2(2.474522113800049e+00f, 1.591999530792236e+01f);
                    vs[2] = new Vector2(6.087579727172852e-01f, 1.375262737274170e+01f);
                    vs[3] = new Vector2(7.533802986145020e-01f, 1.362813091278076e+01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.115987777709961e-01f, 1.375105190277100e+01f);
                    vs[1] = new Vector2(6.207761764526367e-01f, 1.375192642211914e+01f);
                    vs[2] = new Vector2(6.027803421020508e-01f, 7.908406257629395e+00f);
                    vs[3] = new Vector2(7.936034202575684e-01f, 7.907532691955566e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.957516789436340e-01f, -1.650732755661011e+00f);
                    vs[1] = new Vector2(9.957516789436340e-01f, -1.482487916946411e+00f);
                    vs[2] = new Vector2(5.827773213386536e-01f, -1.482487916946411e+00f);
                    vs[3] = new Vector2(5.827773213386536e-01f, -1.650732755661011e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.947141647338867e+00f, -2.402071952819824e+00f);
                    vs[1] = new Vector2(8.059279441833496e+00f, -1.509828448295593e+00f);
                    vs[2] = new Vector2(7.823388099670410e+00f, -1.511801719665527e+00f);
                    vs[3] = new Vector2(8.825365066528320e+00f, -2.518160343170166e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-6.010749340057373e-01f, -9.306051731109619e-01f);
                    vs[1] = new Vector2(-6.010749340057373e-01f, -7.623603343963623e-01f);
                    vs[2] = new Vector2(-7.704768180847168e-01f, -7.623603343963623e-01f);
                    vs[3] = new Vector2(-7.704768180847168e-01f, -9.306051731109619e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.176111412048340e+01f, 5.379695892333984e+00f);
                    vs[1] = new Vector2(2.115560913085938e+01f, 4.700156211853027e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.206690788269043e+01f, 5.190674781799316e+00f);
                    vs[1] = new Vector2(2.176111412048340e+01f, 5.379695892333984e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.267613601684570e+01f, 5.234693050384521e+00f);
                    vs[1] = new Vector2(2.206690788269043e+01f, 5.190674781799316e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.064206027984619e+01f, -2.502783775329590e+00f);
                    vs[1] = new Vector2(1.046836185455322e+01f, -2.334539890289307e+00f);
                    vs[2] = new Vector2(8.825257301330566e+00f, -2.329693317413330e+00f);
                    vs[3] = new Vector2(8.825076103210449e+00f, -2.497937202453613e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.081924438476562e+00f, 1.692813396453857e+00f);
                    vs[1] = new Vector2(7.779927253723145e+00f, 1.692750453948975e+00f);
                    vs[2] = new Vector2(7.742493629455566e+00f, 1.507672309875488e+00f);
                    vs[3] = new Vector2(8.051409721374512e+00f, 1.501664638519287e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.080962371826172e+01f, -2.448249101638794e+00f);
                    vs[1] = new Vector2(1.011487960815430e+01f, -1.714011669158936e+00f);
                    vs[2] = new Vector2(9.993103027343750e+00f, -1.830100059509277e+00f);
                    vs[3] = new Vector2(1.068784713745117e+01f, -2.564337491989136e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.044397354125977e+01f, 4.096230506896973e+00f);
                    vs[1] = new Vector2(1.918307685852051e+01f, 3.306496143341064e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.509812164306641e+01f, 5.975147247314453e+00f);
                    vs[1] = new Vector2(2.769973564147949e+01f, 6.370401859283447e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.819503974914551e+01f, 1.987720131874084e-01f);
                    vs[1] = new Vector2(1.819503974914551e+01f, 3.670166730880737e-01f);
                    vs[2] = new Vector2(1.728530120849609e+01f, 3.670166730880737e-01f);
                    vs[3] = new Vector2(1.728530120849609e+01f, 1.987720131874084e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.987096595764160e+01f, 5.055010795593262e+00f);
                    vs[1] = new Vector2(1.987096595764160e+01f, 5.297784805297852e+00f);
                    vs[2] = new Vector2(1.770611953735352e+01f, 5.297784805297852e+00f);
                    vs[3] = new Vector2(1.770611953735352e+01f, 5.055010795593262e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(7.290708065032959e+00f, 1.699831843376160e+00f);
                    vs[1] = new Vector2(7.265159130096436e+00f, 2.009666919708252e+00f);
                    vs[2] = new Vector2(7.070643901824951e+00f, 1.998311877250671e+00f);
                    vs[3] = new Vector2(7.047934055328369e+00f, 1.699831843376160e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.104205131530762e+01f, 2.114856839179993e-01f);
                    vs[1] = new Vector2(1.104205131530762e+01f, 3.797303140163422e-01f);
                    vs[2] = new Vector2(9.940738677978516e+00f, 3.797303140163422e-01f);
                    vs[3] = new Vector2(9.940738677978516e+00f, 2.114856988191605e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.167337036132812e+01f, -1.643088340759277e+00f);
                    vs[1] = new Vector2(1.155714035034180e+01f, -1.521445751190186e+00f);
                    vs[2] = new Vector2(1.071254825592041e+01f, -2.328456163406372e+00f);
                    vs[3] = new Vector2(1.082877731323242e+01f, -2.450098514556885e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.631994056701660e+01f, -9.485961794853210e-01f);
                    vs[1] = new Vector2(1.631994056701660e+01f, -7.803515195846558e-01f);
                    vs[2] = new Vector2(1.106246852874756e+01f, -7.803515195846558e-01f);
                    vs[3] = new Vector2(1.106246852874756e+01f, -9.485961794853210e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.918307685852051e+01f, 3.306496143341064e+00f);
                    vs[1] = new Vector2(1.703903770446777e+01f, 2.196289300918579e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.509812164306641e+01f, 5.975147247314453e+00f);
                    vs[1] = new Vector2(2.267613601684570e+01f, 5.234693050384521e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.141240882873535e+01f, 1.982975244522095e+00f);
                    vs[1] = new Vector2(2.141240882873535e+01f, 2.151220083236694e+00f);
                    vs[2] = new Vector2(2.050267028808594e+01f, 2.151220083236694e+00f);
                    vs[3] = new Vector2(2.050267028808594e+01f, 1.982975244522095e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.721886634826660e+01f, -4.019510746002197e-01f);
                    vs[1] = new Vector2(1.721886634826660e+01f, -2.337064146995544e-01f);
                    vs[2] = new Vector2(1.630912780761719e+01f, -2.337064146995544e-01f);
                    vs[3] = new Vector2(1.630912780761719e+01f, -4.019510746002197e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.243283462524414e+01f, 2.577025890350342e+00f);
                    vs[1] = new Vector2(2.243283462524414e+01f, 2.745270729064941e+00f);
                    vs[2] = new Vector2(2.152309608459473e+01f, 2.745270729064941e+00f);
                    vs[3] = new Vector2(2.152309608459473e+01f, 2.577025890350342e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.343792152404785e+01f, -9.391410946846008e-01f);
                    vs[1] = new Vector2(2.343792152404785e+01f, -7.708964347839355e-01f);
                    vs[2] = new Vector2(2.212473106384277e+01f, -7.708964347839355e-01f);
                    vs[3] = new Vector2(2.212473106384277e+01f, -9.391410946846008e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.057960510253906e+01f, 1.151917934417725e+01f);
                    vs[1] = new Vector2(4.397109508514404e+00f, 1.200478839874268e+01f);
                    vs[2] = new Vector2(4.355016708374023e+00f, 1.171288490295410e+01f);
                    vs[3] = new Vector2(1.053751182556152e+01f, 1.122727584838867e+01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.866859817504883e+01f, -8.676959037780762e+00f);
                    vs[1] = new Vector2(2.863477706909180e+01f, -8.080880165100098e+00f);
                    vs[2] = new Vector2(-1.436322784423828e+01f, -8.084849357604980e+00f);
                    vs[3] = new Vector2(-1.432940387725830e+01f, -8.680928230285645e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-1.380286026000977e+01f, -8.436588287353516e+00f);
                    vs[1] = new Vector2(-4.977362442016602e+01f, 1.512041282653809e+01f);
                    vs[2] = new Vector2(-5.007194900512695e+01f, 1.460324859619141e+01f);
                    vs[3] = new Vector2(-1.410118389129639e+01f, -8.953750610351562e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(8.198143005371094e+00f, 2.570266246795654e+00f);
                    vs[1] = new Vector2(8.198143005371094e+00f, 7.988479614257812e+00f);
                    vs[2] = new Vector2(7.955369472503662e+00f, 7.988479614257812e+00f);
                    vs[3] = new Vector2(7.955369472503662e+00f, 2.570266246795654e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.741204071044922e+01f, 2.577025890350342e+00f);
                    vs[1] = new Vector2(2.741204071044922e+01f, 2.745270729064941e+00f);
                    vs[2] = new Vector2(2.427039337158203e+01f, 2.745270729064941e+00f);
                    vs[3] = new Vector2(2.427039337158203e+01f, 2.577025890350342e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(6.208030700683594e+01f, 1.814939880371094e+01f);
                    vs[1] = new Vector2(6.167937469482422e+01f, 1.859178543090820e+01f);
                    vs[2] = new Vector2(2.823721694946289e+01f, -8.435064315795898e+00f);
                    vs[3] = new Vector2(2.863815689086914e+01f, -8.877445220947266e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.703903770446777e+01f, 2.196289300918579e+00f);
                    vs[1] = new Vector2(1.560352897644043e+01f, 1.560027003288269e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.560352897644043e+01f, 1.560027003288269e+00f);
                    vs[1] = new Vector2(1.428396320343018e+01f, 1.043329477310181e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.835318946838379e+01f, 6.207946777343750e+00f);
                    vs[1] = new Vector2(2.835318946838379e+01f, 6.376191616058350e+00f);
                    vs[2] = new Vector2(2.769922447204590e+01f, 6.376191616058350e+00f);
                    vs[3] = new Vector2(2.769922447204590e+01f, 6.207946777343750e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 8.000000119209290e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.932248878479004e+01f, 7.928324341773987e-01f);
                    vs[1] = new Vector2(1.932248878479004e+01f, 9.610770344734192e-01f);
                    vs[2] = new Vector2(1.841275024414062e+01f, 9.610770344734192e-01f);
                    vs[3] = new Vector2(1.841275024414062e+01f, 7.928324341773987e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.923006057739258e+00f, 5.839899182319641e-01f);
                    vs[1] = new Vector2(8.060965538024902e+00f, 1.171397447586060e+00f);
                    vs[2] = new Vector2(8.013838768005371e+00f, 9.864833950996399e-01f);
                    vs[3] = new Vector2(9.831165313720703e+00f, 4.210482835769653e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(2.115560913085938e+01f, 4.700156211853027e+00f);
                    vs[1] = new Vector2(2.044397354125977e+01f, 4.096230506896973e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);


                    fd.shape = shape;

                    bodies[18].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-3.580528497695923e-02f, -7.826317548751831e-01f);
                bd.angle = 8.054048418998718e-01f;
                bd.linearVelocity = new Vector2(-2.581855468451977e-02f, 0.000000000000000e+00f);
                bd.angularVelocity = 5.164637044072151e-02f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[19] = world.CreateBody(bd);

                {
                    // I think this is the ball on the end of the distance joint
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    CircleShape shape = new CircleShape();
                    shape.Radius = 5.000000000000000e-01f;
                    shape.Center = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);

                    fd.shape = shape;

                    bodies[19].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(8.139089584350586e+00f, -1.215644359588623e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[20] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 2.000000029802322e-01f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(4.230768382549286e-01f, 2.361963689327240e-01f);
                    vs[1] = new Vector2(-4.230768680572510e-01f, 2.361963689327240e-01f);
                    vs[2] = new Vector2(-4.230768680572510e-01f, -2.361963540315628e-01f);
                    vs[3] = new Vector2(4.230767786502838e-01f, -2.361963987350464e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[20].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.655718612670898e+01f, 6.269404888153076e-01f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[21] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.067263782024384e-01f, 8.474133014678955e-01f);
                    vs[1] = new Vector2(-9.170791506767273e-02f, 8.474133014678955e-01f);
                    vs[2] = new Vector2(-9.170791506767273e-02f, -8.474133610725403e-01f);
                    vs[3] = new Vector2(1.067263633012772e-01f, -8.474135994911194e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[21].CreateFixture(fd);
                }
            }
            {
                // This is the stick that's meant to bounce up
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(17.9f, 5.424887180328369e+00f);
                bd.angle = 4.814267158508301e-03f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[22] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.5f;
                    fd.restitution = 0.4f;
                    fd.density = 0.05f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.193307399749756e+00f, -7.795047014951706e-02f);
                    vs[1] = new Vector2(1.193307399749756e+00f, 9.368468075990677e-02f);
                    vs[2] = new Vector2(-1.108393430709839e+00f, 1.137156039476395e-01f);
                    vs[3] = new Vector2(-1.108393430709839e+00f, -9.785395860671997e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[22].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.078072738647461e+01f, 3.009121179580688e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[23] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.067263782024384e-01f, 8.474133014678955e-01f);
                    vs[1] = new Vector2(-9.170791506767273e-02f, 8.474133014678955e-01f);
                    vs[2] = new Vector2(-9.170791506767273e-02f, -8.474133610725403e-01f);
                    vs[3] = new Vector2(1.067263633012772e-01f, -8.474135994911194e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[23].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-2.771063446998596e-01f, -1.387045264244080e+00f);
                bd.angle = 2.316761016845703e-02f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[24] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.316213130950928e-01f, 8.968108892440796e-02f);
                    vs[1] = new Vector2(-4.312384128570557e-01f, 8.968108892440796e-02f);
                    vs[2] = new Vector2(-4.312384128570557e-01f, -8.438628166913986e-02f);
                    vs[3] = new Vector2(9.316212534904480e-01f, -8.438629657030106e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[24].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[25] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(8.646305799484253e-01f, 5.665562152862549e+00f);
                bd.angle = -7.091794908046722e-02f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[26] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-1.415040493011475e+00f, 1.686131954193115e-02f);
                    vs[1] = new Vector2(-1.457947134971619e+00f, 4.792520999908447e-01f);
                    vs[2] = new Vector2(-1.610695600509644e+00f, 4.658881425857544e-01f);
                    vs[3] = new Vector2(-1.567788958549500e+00f, 3.497719764709473e-03f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[26].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(5.343105316162109e+00f, -1.643933355808258e-01f);
                    vs[1] = new Vector2(5.343105316162109e+00f, 4.717625677585602e-02f);
                    vs[2] = new Vector2(-1.553421258926392e+00f, 3.515185415744781e-02f);
                    vs[3] = new Vector2(-1.553421258926392e+00f, -1.764177083969116e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[26].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(5.349596977233887e+00f, 4.414768218994141e-01f);
                    vs[1] = new Vector2(5.197199344635010e+00f, 4.583749771118164e-01f);
                    vs[2] = new Vector2(5.148450374603271e+00f, -3.436088562011719e-03f);
                    vs[3] = new Vector2(5.300848007202148e+00f, -2.033460140228271e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[26].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-4.321518898010254e+00f, 1.640526008605957e+01f);
                bd.angle = 8.054048418998718e-01f;
                bd.linearVelocity = new Vector2(-2.581855468451977e-02f, 0.000000000000000e+00f);
                bd.angularVelocity = 5.164637044072151e-02f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[27] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+01f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    CircleShape shape = new CircleShape();
                    shape.Radius = 5.000000000000000e-01f;
                    shape.Center = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);

                    fd.shape = shape;

                    bodies[27].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.574825096130371e+01f, 3.597975730895996e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[28] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[28].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.678366470336914e+01f, 3.597975730895996e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[29] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[29].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.352297210693359e+01f, 7.127127051353455e-02f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[30] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[30].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.248203182220459e+01f, 7.127127051353455e-02f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[31] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[31].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(5.670880794525146e+00f, 5.606304168701172e+00f);
                bd.angle = 6.462279510498047e+01f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[32] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 6.000000238418579e-01f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    CircleShape shape = new CircleShape();
                    shape.Radius = 2.500000000000000e-01f;
                    shape.Center = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);

                    fd.shape = shape;

                    bodies[32].CreateFixture(fd);
                }
            }
            {
                // This is the bar that's meant to hit the stick
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.079027271270752e+01f, 7.081282138824463e+00f);
                bd.angle = 1.589920759201050e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[33] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(4.786931037902832e+00f, -2.249179184436798e-01f);
                    vs[1] = new Vector2(4.786931037902832e+00f, 1.007060110569000e-01f);
                    vs[2] = new Vector2(-1.553421497344971e+00f, -2.569290995597839e-02f);
                    vs[3] = new Vector2(-1.553421497344971e+00f, -1.220539808273315e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[33].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.559602928161621e+01f, 7.127127051353455e-02f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[34] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921997785568237e-02f, -8.474146723747253e-01f);
                    vs[1] = new Vector2(9.888499975204468e-02f, 8.474121093750000e-01f);
                    vs[2] = new Vector2(-9.954917430877686e-02f, 8.473728895187378e-01f);
                    vs[3] = new Vector2(-9.921428561210632e-02f, -8.474537730216980e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[34].CreateFixture(fd);
                }
            }
            {
                // This is the box that falls down the hole
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(8.146527290344238e+00f, 8.603442192077637e+00f);
                bd.angle = 1.590483069419861e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[35] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.1f;
                    fd.restitution = 0.2f;
                    fd.density = 0.15f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(0.254f, -0.254f);
                    vs[1] = new Vector2(0.254f, 0.254f);
                    vs[2] = new Vector2(-0.254f, 0.254f);
                    vs[3] = new Vector2(-0.254f, -0.254f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[35].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.144438743591309e+01f, 7.127127051353455e-02f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[36] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[36].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.810375213623047e+01f, 6.631551742553711e+00f);
                bd.angle = 6.462279510498047e+01f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[37] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000014901161e-01f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    CircleShape shape = new CircleShape();
                    shape.Radius = 2.500000000000000e-01f;
                    shape.Center = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);

                    fd.shape = shape;

                    bodies[37].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.867184638977051e+01f, 1.818919181823730e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[38] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[38].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.761596107482910e+01f, 1.220154881477356e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[39] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[39].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.873267364501953e+01f, 5.134416103363037e+00f);
                bd.angle = 1.099908113479614e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[40] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.458525538444519e+00f, 6.259308010339737e-02f);
                    vs[1] = new Vector2(-1.458525657653809e+00f, 6.259308010339737e-02f);
                    vs[2] = new Vector2(-1.458525657653809e+00f, -6.259307265281677e-02f);
                    vs[3] = new Vector2(1.458525419235229e+00f, -6.259308755397797e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[40].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 2.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.665431499481201e+00f, -3.425904512405396e-01f);
                    vs[1] = new Vector2(1.665431499481201e+00f, 3.370162248611450e-01f);
                    vs[2] = new Vector2(1.462241888046265e+00f, 3.370162248611450e-01f);
                    vs[3] = new Vector2(1.462241888046265e+00f, -3.425903916358948e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[40].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 2.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(-1.462772727012634e+00f, -3.372440636157990e-01f);
                    vs[1] = new Vector2(-1.462772727012634e+00f, 3.423626720905304e-01f);
                    vs[2] = new Vector2(-1.665962457656860e+00f, 3.423626720905304e-01f);
                    vs[3] = new Vector2(-1.665962457656860e+00f, -3.372441232204437e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[40].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-6.535434722900391e-01f, -1.142022252082825e+00f);
                bd.angle = -1.570796251296997e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[41] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 0.000000000000000e+00f;
                    fd.restitution = 8.000000119209290e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(3.248410224914551e-01f, -8.438623696565628e-02f);
                    vs[1] = new Vector2(3.248410224914551e-01f, 8.968114852905273e-02f);
                    vs[2] = new Vector2(-3.142510652542114e-01f, 8.968114852905273e-02f);
                    vs[3] = new Vector2(-3.142510652542114e-01f, -8.438622206449509e-02f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[41].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[42] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.455838489532471e+01f, 7.127127051353455e-02f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[43] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[43].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[44] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[45] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.470820426940918e+01f, 3.597976446151733e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[46] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[46].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.275662994384766e+01f, 3.592880964279175e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[47] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -4.354978084564209e+00f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -4.354978084564209e+00f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[47].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[48] = world.CreateBody(bd);
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.972773170471191e+01f, 2.411245346069336e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[49] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(9.921713918447495e-02f, 8.474133610725403e-01f);
                    vs[1] = new Vector2(-9.921714663505554e-02f, 8.474133610725403e-01f);
                    vs[2] = new Vector2(-9.921714663505554e-02f, -8.474133014678955e-01f);
                    vs[3] = new Vector2(9.921713173389435e-02f, -8.474134802818298e-01f);
                    shape.Set(vs); // 4);

                    fd.shape = shape;

                    bodies[49].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                // bd.active = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[50] = world.CreateBody(bd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[12];
                jd.bodyB = bodies[7];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-1.419764943420887e-02f, 3.472273647785187e-01f);
                jd.localAnchorB = new Vector2(1.419764943420887e-02f, -3.472273647785187e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[0] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[10];
                jd.bodyB = bodies[3];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(3.915640115737915e-01f, -6.454584002494812e-02f);
                jd.localAnchorB = new Vector2(-6.432041525840759e-02f, -2.964915335178375e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[1] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[5];
                jd.bodyB = bodies[8];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-1.589771127328277e-03f, 1.294880807399750e-01f);
                jd.localAnchorB = new Vector2(1.589295570738614e-03f, -1.294876039028168e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[2] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[2];
                jd.bodyB = bodies[13];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-7.294950541108847e-03f, 3.572863936424255e-01f);
                jd.localAnchorB = new Vector2(7.294473703950644e-03f, -3.572863936424255e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[3] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[10];
                jd.bodyB = bodies[9];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-3.627054393291473e-01f, -6.571095436811447e-02f);
                jd.localAnchorB = new Vector2(8.541004359722137e-02f, -3.014662265777588e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[4] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[4];
                jd.bodyB = bodies[2];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-1.804183274507523e-01f, 1.079236492514610e-01f);
                jd.localAnchorB = new Vector2(8.202061988413334e-03f, -3.192147314548492e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[5] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[6];
                jd.bodyB = bodies[11];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(8.363898377865553e-04f, 1.560461223125458e-01f);
                jd.localAnchorB = new Vector2(-3.581499680876732e-03f, -4.494129493832588e-02f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[6] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[3];
                jd.bodyB = bodies[1];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(7.003130018711090e-02f, 2.955459952354431e-01f);
                jd.localAnchorB = new Vector2(2.619568957015872e-03f, -3.052060604095459e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[7] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[10];
                jd.bodyB = bodies[5];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-2.784118754789233e-03f, 1.501919031143188e-01f);
                jd.localAnchorB = new Vector2(1.020645722746849e-03f, -1.252421736717224e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[8] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[4];
                jd.bodyB = bodies[12];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(1.821715533733368e-01f, 1.166393160820007e-01f);
                jd.localAnchorB = new Vector2(1.562742702662945e-02f, -3.138234615325928e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[9] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[8];
                jd.bodyB = bodies[4];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-2.156140282750130e-03f, 1.297037452459335e-01f);
                jd.localAnchorB = new Vector2(2.155664609745145e-03f, -1.297032535076141e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[10] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[11];
                jd.bodyB = bodies[10];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(3.814474985119887e-05f, 9.424854815006256e-02f);
                jd.localAnchorB = new Vector2(2.223334275186062e-02f, -1.703564822673798e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[11] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[9];
                jd.bodyB = bodies[14];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-4.893289506435394e-02f, 2.880721986293793e-01f);
                jd.localAnchorB = new Vector2(1.211793627589941e-03f, -3.030030131340027e-01f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerAngle = -8.726646006107330e-02f;
                jd.upperAngle = 8.726646006107330e-02f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[12] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[40];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(2.873267364501953e+01f, 5.134416103363037e+00f);
                jd.localAnchorB = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[13] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[41];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(-6.608279347419739e-01f, -8.509724140167236e-01f);
                jd.localAnchorB = new Vector2(-2.910498380661011e-01f, -7.284440565854311e-03f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[14] = world.CreateJoint(jd);
            }
            {
                DistanceJointDef jd = new DistanceJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[19];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(4.595311164855957e+00f, -1.409123420715332e+00f);
                jd.localAnchorB = new Vector2(4.783708136528730e-03f, 9.254086180590093e-04f);
                jd.length = 4.671227455139160e+00f;
                jd.frequencyHz = 0.000000000000000e+00f;
                jd.dampingRatio = 0.000000000000000e+00f;
                joints[15] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[33];
                jd.collideConnected = true;
                jd.localAnchorA = new Vector2(1.085271549224854e+01f, 5.685722351074219e+00f);
                jd.localAnchorB = new Vector2(-1.397351622581482e+00f, -8.028738200664520e-02f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[16] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[41];
                jd.bodyB = bodies[24];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(2.482175827026367e-01f, -1.928825047798455e-03f);
                jd.localAnchorB = new Vector2(-3.783383965492249e-01f, 5.571336951106787e-03f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[17] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[16];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(7.153423309326172e+00f, 1.670340776443481e+00f);
                jd.localAnchorB = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[18] = world.CreateJoint(jd);
            }
            {
                PrismaticJointDef jd = new PrismaticJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[20];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(8.139089584350586e+00f, -1.215644359588623e+00f);
                jd.localAnchorB = new Vector2(1.000000000000000e+00f, 0.000000000000000e+00f);
                jd.localAxisA = new Vector2(1.000000000000000e+00f, 0.000000000000000e+00f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = true;
                jd.lowerTranslation = -6.154879093170166e+00f;
                jd.upperTranslation = 1.008777618408203e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorForce = 0.000000000000000e+00f;
                joints[19] = world.CreateJoint(jd);
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[18];
                jd.bodyB = bodies[26];
                jd.collideConnected = true;
                jd.localAnchorA = new Vector2(1.495088100433350e+00f, 5.568014621734619e+00f);
                jd.localAnchorB = new Vector2(6.357848644256592e-01f, -5.262904614210129e-02f);
                jd.referenceAngle = 0.000000000000000e+00f;
                jd.enableLimit = false;
                jd.lowerAngle = 0.000000000000000e+00f;
                jd.upperAngle = 0.000000000000000e+00f;
                jd.enableMotor = false;
                jd.motorSpeed = 0.000000000000000e+00f;
                jd.maxMotorTorque = 0.000000000000000e+00f;
                joints[20] = world.CreateJoint(jd);
            }

            return world;
        }
    }
}
