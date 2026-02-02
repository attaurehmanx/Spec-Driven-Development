# AI Agent Service - Testing Quick Reference

## TL;DR

```bash
cd "Z:\phse 33\backend"
python run_all_tests.py
```

That's it. All tests are automated. Just run and review results.

---

## What Gets Tested

1. **T020**: Agent calls add_task tool correctly
2. **T021**: Agent calls list_tasks tool correctly
3. **T026**: Agent resolves pronouns ("Mark it as done")
4. **T027**: Agent identifies list references ("Delete the first one")
5. **T035**: Agent explains errors naturally
6. **T036**: Agent asks clarifying questions
7. **T041**: Agent interprets "today" correctly
8. **T042**: Agent interprets "tomorrow" correctly

---

## Success Criteria

All tests should show:
- ✓ Status: "completed"
- ✓ Iterations: 1-5
- ✓ Time: <3 seconds
- ✓ Validation: PASS

---

## If Tests Fail

1. Check `.env` file has valid `GEMINI_API_KEY`
2. Verify database is accessible
3. Ensure test user exists or use existing user ID
4. Review error messages in test output
5. See `TESTING_GUIDE.md` for detailed troubleshooting

---

## Documentation

- **Quick Start**: `MANUAL_TEST_INSTRUCTIONS.md`
- **Comprehensive Guide**: `TESTING_GUIDE.md`
- **Detailed Report**: `TESTING_COMPLETION_REPORT.md`
- **Action Plan**: `ACTION_PLAN.md`

---

## After Testing

1. Mark tasks T020-T042, T049 complete in `tasks.md`
2. Update `quickstart.md` with actual outputs (T050)
3. Commit changes
4. Proceed to Chat API integration

---

**Ready**: Execute `python run_all_tests.py` to begin
