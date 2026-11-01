        ProfileParams p;
        double D = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
        int sideK = I(0, 9);
        p.side = sideK < 5 ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 2) == 0 ? 0.0 : U(0.0, 0.5);
        p.passes.startDepth = I(0, 3) == 0 ? U(0, 3) : 0.0;
        p.passes.cutDepth = U(2, 20);
        p.passes.passDepth = I(0, 3) == 0 ? U(0.4, 1.5) : U(1, 8);
        if (I(0, 5) == 0) p.passes.lastPassThickness = U(0.2, 1.0);
        p.separateLastPass = forceSep >= 0 ? forceSep : I(0, 1);
        p.lastPassAllowance = I(0, 5) == 0 ? U(0.0, 0.1) : U(0.05, 1.0);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = true;
        p.tabs.length = I(0, 1) ? U(0.3, 4) : U(1, 20);
        if (std::getenv("TLMIN")) p.tabs.length = U(std::atof(std::getenv("TLMIN")), std::atof(std::getenv("TLMAX")));
        p.tabs.thickness = I(0, 7) == 0 ? U(p.passes.cutDepth, p.passes.cutDepth + 5) : U(0.3, std::min(6.0, p.passes.cutDepth));
        p.tabs.spacing = I(0, 3) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(0, 1) ? 2 * I(0, 4) + 1 : I(1, 9);
        p.tabs.distance = U(10, 120);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 2) == 0;
        p.ramp.type = (RampType)I(0, 3);
        p.ramp.distance = U(1, 40);
        p.ramp.useAngle = I(0, 2) == 0;
        p.ramp.angle = U(1, 30) * PI / 180;
        p.ramp.onLeadIn = I(0, 3) == 0;
        p.leads.type = (LeadType)I(0, 2);
        p.leads.length = U(0.5, 15);
        p.leads.angle = U(0.05, PI / 2);
        p.leads.radius = U(0.5, 12);
        p.leads.sweep = U(5, 90) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 20) : 0.0;
        p.startAt = (StartAt)I(0, 2);
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-50, 50), U(-50, 50)};
        p.order.leftToRight = I(0, 3) == 0;
        p.order.grid = I(0, 3) == 0;
        bool useV = false; double vang = 90;
        std::vector<Contour> cs;
        int f = fam == 0 ? I(1, 9) : fam;
        double R = U(8, 35);
        switch (f) {
            case 1: cs.push_back(Contour::circle({0, 0}, U(4, 30), I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW)); break;  // concave (Inside) / convex (Outside) arcs
            case 2: { int k = I(0, 2); if (k == 0) cs.push_back(flower({0, 0}, U(2.5, 10), 2 * I(2, 5))); else if (k == 1) cs.push_back(gear({0, 0}, U(8, 25), I(6, 20), U(1, 6))); else cs.push_back(spikes({0, 0}, U(8, 25), I(3, 8))); break; }
            case 3: {  // nested
                double W = U(50, 120), H = U(40, 90);
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
                int n = I(1, 3);
                for (int i = 0; i < n; ++i) { double rr = U(4, 15); Vec2 c{U(rr + 0.5, W - rr - 0.5), U(rr + 0.5, H - rr - 0.5)}; cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.7}, {c.x + rr, c.y + rr * 0.7}})); }
                if (I(0, 1)) cs.push_back(Contour::circle({W / 2, H / 2}, std::min(W, H) * 0.1));
                break;
            }
            case 4: cs.push_back(bezBlob({0, 0}, R, I(4, 9))); break;
            case 5: cs.push_back(I(0, 1) ? anyOpen({0, 0}, R) : bezWave({0, 0}, U(30, 120), I(2, 6))); if (I(0, 3) == 0) p.side = ProfileSide::On; break;
            case 6: cs.push_back(I(0, 1) ? Contour::rectangle(geometry::Box2{{0, 0}, {U(10, 120), U(8, 80)}}) : chamfered({0, 0}, U(30, 100), U(20, 70), U(0.5, 10))); if (I(0, 1)) { p.startAt = StartAt::ClosestOnBoundingBox; } break;
            case 7: cs.push_back(ellipse({0, 0}, U(5, 40), U(3, 20), U(0, PI))); break;
            case 8: {  // V-bit
                useV = true; vang = std::vector<double>{60, 90, 120}[(size_t)I(0, 2)];
                cs.push_back(anyClosed({0, 0}, R));
                p.passes.cutDepth = U(1, 6);
                p.tabs.thickness = U(0.2, p.passes.cutDepth);
                p.corners.sharpExternal = I(0, 1); p.corners.sharpInternal = I(0, 1);
                break;
            }
            case 99: { int vd = 0; cs.clear(); p = ProfileParams{}; readScene(std::getenv("SCENE"), cs, p, D, vd); if (vd) { useV = true; vang = vd; D /= 2; } break; }
            case 23: {  // N1: rectangle, 4 tabs centred on the corners
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}})); p = ProfileParams{};
                D = std::vector<double>{3.175, 6.0, 12.0}[(size_t)(sc % 3)]; p.side = (sc / 3) % 2 ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = (sc / 6) % 2 ? 0.2 : 0.0;
                p.separateLastPass = (sc / 12) % 2; p.lastPassAllowance = 0.3;
                p.tabs.enabled = true; p.tabs.length = 0.25 + 0.25 * ((sc / 24) % 20); p.tabs.thickness = 1.5; p.tabs.count = 4;
                p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre; p.passes.cutDepth = 6; p.passes.passDepth = 3;
                break;
            }
            case 20: {  // round-1 b5circle (band 26010 fam 10 scene 54)
                cs.push_back(Contour::circle({0, 0}, 19.679080419874751)); p = ProfileParams{}; D = 8.0;
                p.side = ProfileSide::Inside; p.allowance = 0.35808273924283024; p.passes.startDepth = 2.5890622067618838; p.passes.cutDepth = 6.6827760460028767; p.passes.passDepth = 5.5108253589101883;
                p.separateLastPass = true; p.lastPassAllowance = 0.89243563469697218; p.tabs.enabled = true; p.tabs.count = 1; p.tabs.length = 0.62026736706984131; p.tabs.thickness = 4.3090084694084512;
                p.ramp.type = RampType::ZigZag; p.ramp.distance = 36.75464170141808; p.leads.type = LeadType::Circular; p.leads.length = 8.2550718220314288; p.leads.angle = 0.69261347705451748; p.leads.radius = 6.3947391786514576; p.leads.sweep = 0.15589211200284489;
                p.startAt = StartAt::Optimise; p.startAnchor = BoxAnchor::TopRight; p.startPosition = {11.026666867948485, -6.302635438741838};
                break;
            }
            case 21: {  // round-1 b5blob
                const double B[8][8] = {
        {24.528297700458356, -0.075131575306200141, 24.589931499642784, 5.5471493574607127, 21.023356708879575, 14.160905540386025, 16.933676131720073, 17.50028364950516},
        {16.933676131720073, 17.50028364950516, 12.843995554560571, 20.839661758624295, 4.6004164157089189, 21.080710980455006, -0.009785762498662717, 19.961137079408608},
        {-0.009785762498662717, 19.961137079408608, -4.6199879407062436, 18.841563178362211, -7.0160096946899637, 14.103805991146414, -10.727536937525418, 10.782840243226778},
        {-10.727536937525418, 10.782840243226778, -14.439064180360873, 7.461874495307141, -21.381104878470005, 4.4245520465845924, -22.278949219511389, 0.035342591890786854},
        {-22.278949219511389, 0.035342591890786854, -23.176793560552774, -4.3538668628030184, -19.86863318804566, -13.108168954594763, -16.114602983773732, -15.552416484936053},
        {-16.114602983773732, -15.552416484936053, -12.360572779501807, -17.996664015277343, -5.2011807139443693, -14.516645013130244, 0.24523200612016866, -14.630142590156956},
        {0.24523200612016866, -14.630142590156956, 5.6916447261847072, -14.743640167183667, 12.516695720890473, -18.65923711623811, 16.563873336613504, -16.233401947096318},
        {16.563873336613504, -16.233401947096318, 20.611050952336534, -13.807566777954527, 24.466663901273929, -5.6974125080731124, 24.528297700458356, -0.075131575306200141}};
                Contour c; c.closed = true;
                for (auto& bb : B) c.append(geometry::BezierSpan{{bb[0], bb[1]}, {bb[2], bb[3]}, {bb[4], bb[5]}, {bb[6], bb[7]}});
                cs.push_back(c); p = ProfileParams{}; D = 12.0;
                p.side = ProfileSide::Inside; p.direction = CutDirection::Climb; p.allowance = 0.49190493333618557; p.passes.cutDepth = 18.165175165326691; p.passes.passDepth = 5.9478725204244043;
                p.separateLastPass = true; p.lastPassAllowance = 0.95809446752021477; p.tabs.enabled = true; p.tabs.count = 1; p.tabs.length = 1.4386404699535462; p.tabs.thickness = 3.576056744455637; p.tabs.avoidCorners = true;
                p.leads.type = LeadType::Straight; p.leads.length = 14.197196335998006; p.leads.angle = 1.1481504767893238; p.leads.leadOut = true;
                p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::TopLeft; p.startPosition = {39.967753505838715, -23.008829543971238}; p.order.leftToRight = true; p.order.grid = true;
                break;
            }
            default: cs.push_back(anyClosed({0, 0}, R)); if (I(0, 2) == 0) cs.push_back(anyClosed({3.5 * 35, 0}, U(8, 25))); break;
        }
        if (std::getenv("ONP") && f != 99) { bool anyOpen0 = false; for (auto& c : cs) anyOpen0 |= !c.closed; if (!anyOpen0 && I(0, std::atoi(std::getenv("ONP")) - 1) == 0) p.side = ProfileSide::On; }
        if (std::getenv("NEGA") && f != 99) {  // T-0127 coder: negative allowances (In/Out down to -r, On +-0.5)
            double rr0 = useV ? 0.0 : D / 2; int k = I(0, 7); double u0 = U(0, 1);
            if (!useV) {
                if (p.side == ProfileSide::On) p.allowance = k < 5 ? -0.5 + u0 : 0.0;
                else if (k == 0) p.allowance = -rr0;
                else if (k <= 3) p.allowance = -rr0 + 0.5 * u0;
                else p.allowance = -rr0 * u0;
            }
        }
