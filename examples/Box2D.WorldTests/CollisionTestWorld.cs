//  Source code dump of Box2D scene: collidetest.rube
//
//  Created by R.U.B.E 1.7.4
//  Using Box2D version 2.3.0
//  Mon June 22 2020 11:16:26
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
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.WorldTests
{
    [ExcludeFromCodeCoverage]
    public static class CollisionTestWorld
    {
        public static World CreateWorld()
        {
            var g = new Vector2(0.000000000000000e+00f, -1.000000000000000e+01f);
            var world = new World(g);
            var bodies = new Body[4];
            var joints = new Joint[1];

            {
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
                //bd.active          = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[0] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 2.000000029802322e-01f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (ushort)(1);
                    fd.filter.maskBits = (ushort)(65535);
                    fd.filter.groupIndex = (short)(0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(3.446016013622284e-01f, -2.423547506332397e-01f);
                    vs[1] = new Vector2(3.446016013622284e-01f, 2.525457143783569e-01f);
                    vs[2] = new Vector2(-3.599468171596527e-01f, 2.588300704956055e-01f);
                    vs[3] = new Vector2(-3.599468171596527e-01f, -2.446277290582657e-01f);
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
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
                //bd.active          = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[1] = world.CreateBody(bd);

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
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[1].CreateFixture(fd);
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
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[1].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.108404064178467e+01f, 6.898103237152100e+00f);
                bd.angle = 1.589920759201050e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                //bd.active          = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[2] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 5.000000000000000e-01f;
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
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[2].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(1.796243667602539e+01f, 5.424887180328369e+00f);
                bd.angle = 4.814267158508301e-03f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.awake = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                //bd.active          = true;
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[3] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 5.000000000000000e-01f;
                    fd.restitution = 6.999999880790710e-01f;
                    fd.density = 5.000000074505806e-02f;
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
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[3].CreateFixture(fd);
                }
            }
            {
                RevoluteJointDef jd = new RevoluteJointDef();
                jd.bodyA = bodies[1];
                jd.bodyB = bodies[2];
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
                joints[0] = world.CreateJoint(jd);
            }

            return world;
        }
    }
}
