#include <map>
#include <random>

#include "doctest/doctest.h"
#include "scene/ecs.hpp"
#include "scene/game_loop.hpp"
#include "scene/node.hpp"

using namespace ifg;

TEST_CASE("signals: connect, emit, disconnect, mutation during emit") {
    Signal<int> s;
    int sum = 0;
    auto a = s.connect([&](int v) { sum += v; });
    Signal<int>::Id b = 0;
    b = s.connect([&](int v) {
        sum += 10 * v;
        s.disconnect(b);  // disconnect self during emit
    });
    s.emit(1);
    CHECK(sum == 11);
    s.emit(1);
    CHECK(sum == 12);
    CHECK(s.size() == 1);
    // slot added during emit runs only from the next emit
    s.connect([&](int) { s.connect([&](int) { sum += 1000; }); });
    s.emit(0);
    CHECK(sum == 12);
    s.emit(0);
    CHECK(sum == 1012);
    CHECK(s.disconnect(a));
    CHECK_FALSE(s.disconnect(a));
    // a slot disconnected by an earlier slot in the same emit is skipped
    Signal<> t;
    int calls = 0;
    Signal<>::Id second = 0;
    t.connect([&] { t.disconnect(second); });
    second = t.connect([&] { ++calls; });
    t.emit();
    CHECK(calls == 0);
}

TEST_CASE("node tree: ownership, paths, unique names, find") {
    Node root("Root");
    int added = 0;
    root.child_added.connect([&](Node*) { ++added; });
    Node* a = root.emplace_child<Node>("A");
    Node* b = a->emplace_child<Node>("B");
    Node* a2 = root.emplace_child<Node>("A");
    CHECK(a2->name() == "A2");
    CHECK(added == 2);
    CHECK(b->path() == "/Root/A/B");
    CHECK(root.find("A/B") == b);
    CHECK(b->find("..") == a);
    CHECK(b->find("../../A2") == a2);
    CHECK(b->find("/Root/A2") == a2);
    CHECK(root.find("Nope") == nullptr);
    CHECK(root.find("/Wrong/A") == nullptr);
    CHECK(b->root() == &root);
    CHECK(root.subtree_size() == 4);
    auto owned = root.remove_child(a);
    CHECK(owned->parent() == nullptr);
    CHECK(root.subtree_size() == 2);
    CHECK(owned->find("B") == b);
}

TEST_CASE("node global transforms compose") {
    Node root("R");
    root.transform.position = {10, 0, 0};
    root.transform.rotation = Quat::from_axis_angle({0, 1, 0}, kPi / 2);
    Node* c = root.emplace_child<Node>("C");
    c->transform.position = {0, 0, -1};
    Vec3 p = c->global_transform().position;
    CHECK(approx(p, Vec3(9, 0, 0)));
}

namespace {
struct Counter : Node {
    explicit Counter(std::string n, int* log) : Node(std::move(n)), log_(log) {}
    void process(float) override { ++*log_; }
    int* log_;
};
}  // namespace

TEST_CASE("process propagation and deferred free") {
    int count = 0;
    Node root("Root");
    auto* c1 = root.emplace_child<Counter>("C1", &count);
    c1->emplace_child<Counter>("C1a", &count);
    root.emplace_child<Counter>("C2", &count);
    float total = 0;
    root.processed.connect([&](float dt) { total += dt; });
    root.propagate_process(0.5f);
    CHECK(count == 3);
    CHECK(total == 0.5f);
    auto token = c1->lifetime_token();
    c1->queue_free();
    root.propagate_process(0.5f);
    CHECK(count == 6);  // still processed this frame
    CHECK(token.expired());
    CHECK(root.find("C1") == nullptr);
    root.propagate_process(0.5f);
    CHECK(count == 7);
}

TEST_CASE("named user signals on nodes") {
    Node n("N");
    float got = 0;
    auto id = n.connect("hit", [&](float v) { got += v; });
    n.emit("hit", 2.5f);
    n.emit("other", 100.0f);
    CHECK(got == 2.5f);
    CHECK(n.disconnect("hit", id));
    n.emit("hit", 1.0f);
    CHECK(got == 2.5f);
}

namespace {
struct Pos { float x, y; };
struct Vel { float x, y; };
struct Tag { int v; };
}  // namespace

TEST_CASE("ECS sparse sets: emplace/remove/iterate/generations") {
    Registry reg;
    std::vector<Entity> es;
    for (int i = 0; i < 100; ++i) {
        Entity e = reg.create();
        es.push_back(e);
        reg.emplace<Pos>(e, float(i), 0.0f);
        if (i % 2 == 0) reg.emplace<Vel>(e, 1.0f, 2.0f);
        if (i % 3 == 0) reg.emplace<Tag>(e, i);
    }
    int n = 0;
    reg.each<Pos, Vel>([&](Entity, Pos& p, Vel& v) {
        p.x += v.x;
        ++n;
    });
    CHECK(n == 50);
    CHECK(reg.get<Pos>(es[2]).x == 3.0f);
    CHECK(reg.get<Pos>(es[1]).x == 1.0f);
    n = 0;
    reg.each<Pos, Vel, Tag>([&](Entity, Pos&, Vel&, Tag& t) {
        CHECK(t.v % 6 == 0);
        ++n;
    });
    CHECK(n == 17);
    // remove keeps others intact (swap-and-pop)
    reg.remove<Pos>(es[0]);
    CHECK_FALSE(reg.has<Pos>(es[0]));
    CHECK(reg.get<Pos>(es[99]).x == 99.0f);
    CHECK(reg.pool<Pos>().size() == 99);
    // destroy + recycle bumps generation: stale handles are rejected
    reg.destroy(es[5]);
    CHECK_FALSE(reg.alive(es[5]));
    CHECK_FALSE(reg.has<Pos>(es[5]));
    Entity r = reg.create();
    CHECK(entity_index(r) == entity_index(es[5]));
    CHECK(entity_gen(r) == entity_gen(es[5]) + 1);
    CHECK(reg.alive(r));
    CHECK_FALSE(reg.has<Pos>(r));
    CHECK(reg.try_get<Pos>(es[5]) == nullptr);
    CHECK(reg.alive_count() == 100);
}

TEST_CASE("ECS randomized against a reference map") {
    std::mt19937 rng(9);
    Registry reg;
    std::map<Entity, int> ref;
    for (int it = 0; it < 5000; ++it) {
        int op = rng() % 3;
        if (op == 0 || ref.empty()) {
            Entity e = reg.create();
            int v = static_cast<int>(rng() % 1000);
            reg.emplace<Tag>(e, v);
            ref[e] = v;
        } else {
            auto itr = ref.begin();
            std::advance(itr, rng() % ref.size());
            if (op == 1) reg.destroy(itr->first);
            else reg.remove<Tag>(itr->first);
            ref.erase(itr);
        }
    }
    CHECK(reg.pool<Tag>().size() == ref.size());
    for (auto& [e, v] : ref) CHECK(reg.get<Tag>(e).v == v);
}

TEST_CASE("fixed-step loop is deterministic regardless of frame slicing") {
    auto simulate = [](const std::vector<double>& frames) {
        FixedStepLoop loop(1.0 / 60.0, 1000);
        double x = 0, v = 1;
        loop.on_fixed_update = [&](double dt, std::uint64_t) {
            v -= 9.81 * dt;
            x += v * dt;
        };
        for (double f : frames) loop.advance(f);
        return std::make_tuple(loop.tick(), x, v);
    };
    std::mt19937 rng(3);
    std::vector<double> jittery, steady(120, 1.0 / 60.0);
    double total = 0;
    while (total < 2.0 - 1.0 / 30) {
        double f = std::uniform_real_distribution<double>(0.001, 0.05)(rng);
        jittery.push_back(f);
        total += f;
    }
    jittery.push_back(2.0 - total);
    auto a = simulate(steady), b = simulate(jittery);
    CHECK(std::get<0>(a) == 120);
    CHECK(std::get<0>(a) == std::get<0>(b));
    CHECK(std::get<1>(a) == std::get<1>(b));  // bit-identical state
    CHECK(std::get<2>(a) == std::get<2>(b));
}

TEST_CASE("fixed-step loop spiral-of-death guard and alpha") {
    FixedStepLoop loop(0.01, 4);
    int n = 0;
    loop.on_fixed_update = [&](double, std::uint64_t) { ++n; };
    CHECK(loop.advance(1.0) == 4);
    CHECK(loop.dropped_time() > 0.9);
    CHECK(loop.advance(0.015) == 1);
    CHECK(loop.alpha() == doctest::Approx(0.5));
    CHECK(n == 5);
}
