// Build the HTML first with build_research_notebook.py. Requires Playwright/Chromium.
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
(async () => {
  const root = path.resolve(__dirname, '..');
  const browser = await chromium.launch({ headless: true });
  try {
    const page = await browser.newPage();
    await page.goto(pathToFileURL(path.join(root, 'docs/notes/notebook.html')).href);
    await page.evaluate(() => document.fonts.ready);
    await page.waitForFunction(() => [...document.images].every(i => i.complete && i.naturalWidth));
    const failures = await page.locator('.page').evaluateAll(pages => pages.flatMap((p, i) => {
      const content = p.querySelector('main').getBoundingClientRect();
      const footer = p.querySelector('footer').getBoundingClientRect();
      return content.bottom + 10 > footer.top || p.scrollHeight > p.clientHeight ? [i + 1] : [];
    }));
    if (failures.length) throw new Error(`Page content exceeds layout budget: ${failures.join(', ')}`);
    await page.pdf({
      path: path.join(root, 'docs/notes/ffc-research-notebook.pdf'),
      preferCSSPageSize: true, printBackground: true, tagged: true, outline: true,
    });
    console.log('Exported research notebook; all page-layout checks passed.');
  } finally {
    await browser.close();
  }
})();
