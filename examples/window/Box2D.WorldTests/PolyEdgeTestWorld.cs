//  Source code dump of Box2D scene: polyedgetest.rube
//
//  Created by R.U.B.E 1.7.4
//  Using Box2D version 2.3.0
//  Mon June 22 2020 12:12:35
//
//  This code is originally intended for use in the Box2D testbed,
//  but you can easily use it in other applications by providing
//  a b2World for use as the 'm_world' variable in the code below.

using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.WorldTests
{
    [ExcludeFromCodeCoverage]
    public static class PolyEdgeTestWorld
    {
        public static World CreateWorld()
        {
            var gravity = new Vector2(0.000000000000000e+00f, -1.000000000000000e+01f);
            var world = new World(gravity);
            var bodies = new Body[2];

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
                bodies[0] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 1.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.622321891784668e+01f, 4.735573291778564e+00f);
                    vs[1] = new Vector2(1.601119422912598e+01f, 4.735989570617676e+00f);
                    vs[2] = new Vector2(1.602764320373535e+01f, 3.814474105834961e+00f);
                    vs[3] = new Vector2(1.621816253662109e+01f, 3.825283050537109e+00f);
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(1.918307685852051e+01f, 3.306496143341064e+00f);
                    vs[1] = new Vector2(1.703903770446777e+01f, 2.196289300918579e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(2.044397354125977e+01f, 4.096230506896973e+00f);
                    vs[1] = new Vector2(1.918307685852051e+01f, 3.306496143341064e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(2.176111412048340e+01f, 5.379695892333984e+00f);
                    vs[1] = new Vector2(2.115560913085938e+01f, 4.700156211853027e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(2.206690788269043e+01f, 5.190674781799316e+00f);
                    vs[1] = new Vector2(2.176111412048340e+01f, 5.379695892333984e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(1.703903770446777e+01f, 2.196289300918579e+00f);
                    vs[1] = new Vector2(1.560352897644043e+01f, 1.560027003288269e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(1.560352897644043e+01f, 1.560027003288269e+00f);
                    vs[1] = new Vector2(1.428396320343018e+01f, 1.043329477310181e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    EdgeShape shape = new EdgeShape();
                    Vector2[] vs = new Vector2[4];
                    ;
                    vs[0] = new Vector2(2.115560913085938e+01f, 4.700156211853027e+00f);
                    vs[1] = new Vector2(2.044397354125977e+01f, 4.096230506896973e+00f);

                    vs[2] = new Vector2(4.565430396770254e-41f, 0.000000000000000e+00f);
                    vs[3] = new Vector2(0.000000000000000e+00f, 1.216642260551453e-01f);
                    shape.SetTwoSided(vs[0], vs[1]);
                    ;

                    fd.shape = shape;

                    bodies[0].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(2.079453659057617e+01f, 7.627632617950439e+00f);
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
                bodies[1] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 5.000000000000000e-01f;
                    fd.restitution = 6.999999880790710e-01f;
                    fd.density = 5.000000074505806e-02f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    PolygonShape shape = new PolygonShape();
                    Vector2[] vs = new Vector2[4];
                    vs[0] = new Vector2(1.193307399749756e+00f, -7.795047014951706e-02f);
                    vs[1] = new Vector2(1.193307399749756e+00f, 9.368468075990677e-02f);
                    vs[2] = new Vector2(-1.108393430709839e+00f, 1.137156039476395e-01f);
                    vs[3] = new Vector2(-1.108393430709839e+00f, -9.785395860671997e-02f);
                    shape.Set(vs);

                    fd.shape = shape;

                    bodies[1].CreateFixture(fd);
                }
            }

            return world;
        }
    }
}
