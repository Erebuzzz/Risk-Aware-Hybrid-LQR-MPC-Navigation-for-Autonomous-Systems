import asyncio
from playwright.async_api import async_playwright
import os

async def generate_pdf():
    async with async_playwright() as p:
        # Launch headless browser
        browser = await p.chromium.launch()
        page = await browser.new_page()
        
        # Get the absolute path of the HTML file
        html_file = os.path.abspath("Report_AsyncMPC_PID.html")
        file_url = f"file:///{html_file.replace(os.sep, '/')}"
        
        print(f"Loading {file_url} ...")
        # Go to the local HTML file
        await page.goto(file_url, wait_until='networkidle')
        
        # Wait a bit for MathJax to render
        print("Waiting for MathJax to render...")
        await page.wait_for_timeout(3000)
        
        # Generate the PDF
        pdf_path = "Report_AsyncMPC_PID.pdf"
        print(f"Saving PDF to {pdf_path} ...")
        await page.pdf(
            path=pdf_path,
            format="A4",
            print_background=True,
            margin={"top": "20mm", "bottom": "20mm", "left": "20mm", "right": "20mm"}
        )
        
        await browser.close()
        print("PDF Generation Complete!")

if __name__ == "__main__":
    asyncio.run(generate_pdf())
