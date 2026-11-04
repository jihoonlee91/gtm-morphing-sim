classdef SmokeTest < matlab.unittest.TestCase
    % Fast checks that a fresh MATLAB can load, trim, and fly the GTM-T2 simulation.
    % Run from the repository root:  results = runtests("tests")
    % CI runs this on every push (.github/workflows/matlab.yml).

    properties (Constant)
        DesignDir = fullfile(fileparts(fileparts(mfilename("fullpath"))), "gtm_design")
    end

    methods (TestClassSetup)
        function runSetup(testCase)
            here = pwd;
            testCase.addTeardown(@() cd(here));
            testCase.addTeardown(@() bdclose("all"));
            cd(testCase.DesignDir);
            evalin("base", "setup");
        end
    end

    methods (Test)
        function trimsAndFliesLevel(testCase)
            % Same steps as main.m: nominal starting point, trim at 100 kt level flight, simulate.
            cd(testCase.DesignDir);
            loadmws(init_design("GTM_T2"));
            [MWS, ~, ~, err] = trimgtm(struct("tas", 100, "gamma", 0));
            testCase.verifyLessThan(err, 1e-3, "trim residual too large");
            loadmws(MWS);
            [t, x] = sim("gtm_design", [0 5]);
            testCase.verifyGreaterThanOrEqual(t(end), 5 - 1e-9);
            testCase.verifyTrue(all(isfinite(x(:))), "simulation produced non-finite states");
        end
    end
end
